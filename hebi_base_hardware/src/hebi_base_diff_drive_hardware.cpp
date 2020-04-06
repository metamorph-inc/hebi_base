// Based heavily on https://github.com/husky/husky/blob/melodic-devel/husky_base/src/husky_hardware.cpp

#include "hebi_base_hardware/hebi_base_diff_drive_hardware.h"
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp> 
#include <thread>
#include "Eigen/Dense"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
  const uint8_t NUM_WHEELS = 4;
};

namespace hebi_base_diff_drive_hardware
{

  HebiBaseDiffDriveHardware::HebiBaseDiffDriveHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh)
    //hebi_feedback_(NUM_WHEELS),
    //hebi_command_(NUM_WHEELS)
  {;
    std::string hebi_gains_fname;
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.2032);
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout", polling_timeout_, 10.0);
    private_nh_.param<std::string>("hebi_gains_fname", hebi_gains_fname, "gains/diff_drive/x8_3.xml");

    // Look up HEBI Modules
    std::vector<std::string> hebi_mapping_param_names = {
      "hebi_mapping_front_left_wheel", 
      "hebi_mapping_front_right_wheel", 
      "hebi_mapping_rear_left_wheel", 
      "hebi_mapping_rear_right_wheel"
    };
    std::vector<std::string> hebi_mapping_param_default_values = {
      "DiffDriveBase/FrontLeft", 
      "DiffDriveBase/FrontRight",
      "DiffDriveBase/RearLeft",
      "DiffDriveBase/RearRight"
    };

    std::vector<std::string> hebi_mapping;
    std::vector<std::string> hebi_families;
    std::vector<std::string> hebi_names;
    ROS_INFO("Expected HEBI Module Family/Names:");
    for (int i = 0; i < NUM_WHEELS; i++)  // TODO: could use iterators for more flexibility - J
    {
      hebi_mapping = getHebiFamilyNameFromParam(hebi_mapping_param_names[i], hebi_mapping_param_default_values[i]);
      if (hebi_mapping.size() < 2) {
        // assume HEBI Family is missing
        std::vector<std::string> hebi_mapping_default;
        boost::split(hebi_mapping_default, hebi_mapping_param_default_values[i], boost::is_any_of("/"));
        hebi_families.push_back(hebi_mapping_default[0]);
        hebi_names.push_back(hebi_mapping[0]);
      } else {
        hebi_families.push_back(hebi_mapping[0]);
        hebi_names.push_back(hebi_mapping[1]);
      }
      ROS_INFO("%s: %s/%s", hebi_mapping_param_names[i].c_str(), hebi_families[i].c_str(), hebi_names[i].c_str());
    }

    hebi::Lookup lookup;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printHebiLookup(lookup);
    hebi_group_ = lookup.getGroupFromNames(hebi_families, hebi_names, 10000);
    if (!hebi_group_) {
      ROS_ERROR("HEBI Group not found...");
      // do nothing... for now;
    }
    
    // Set HEBI Gains from file
    hebi::GroupCommand base_gains_command(hebi_group_->size());
    if (!base_gains_command.readGains(hebi_gains_fname)) {
      ROS_WARN("Could not read %s", hebi_gains_fname.c_str());
    } else {
      if (!hebi_group_->sendCommandWithAcknowledgement(base_gains_command)) {
        ROS_WARN("Could not send gains");
      }
    }

    constexpr double feedback_frequency = 100;
    hebi_group_->setFeedbackFrequencyHz(feedback_frequency);
    hebi_feedback_ = std::make_shared<hebi::GroupFeedback>(hebi_group_->size());
    constexpr long command_lifetime = 250;
    hebi_group_->setCommandLifetimeMs(command_lifetime);
    hebi_command_ = std::make_shared<hebi::GroupCommand>(hebi_group_->size());
    cmd_vel_vector_ = Eigen::VectorXd(hebi_group_->size());
    cmd_vel_vector_.setZero();

    // Try to get feedback -- if we don't get a packet in the first N times,
    // something is wrong
    int num_attempts = 0;
    while (!hebi_group_->getNextFeedback(*hebi_feedback_)) {
      if (num_attempts++ > 20) {
        ROS_ERROR("Unable to get HEBI Group Feedback after %s attempts...", std::to_string(num_attempts).c_str());
       // do nothing... for now;
      }
    }

    resetTravelOffset();
    registerControlInterfaces();
  }

  std::string HebiBaseDiffDriveHardware::eigenToString(const Eigen::MatrixXd& mat) {
      std::stringstream ss;
      ss << mat;
      return ss.str();
  }

  /**
   * Get HEBI family / name from ROS Parameter Server
   */ 
  std::vector<std::string> HebiBaseDiffDriveHardware::getHebiFamilyNameFromParam(const std::string &param_name, const std::string &param_default_value) 
  {
    std::string param_value;
    private_nh_.param<std::string>(param_name, param_value, param_default_value);

    // "DiffDriveBase/FrontLeft" --> {"DiffDriveBase", "FrontLeft"}
    std::vector<std::string> param_value_vector;
    boost::split(param_value_vector, param_value, boost::is_any_of("/"));

    return param_value_vector;
  }

  /**
   * Print HEBI module lookup to screen
   */
  void HebiBaseDiffDriveHardware::printHebiLookup(hebi::Lookup &hebi_lookup)
  {
    // Take snapshot and print to the screen
    auto entry_list = hebi_lookup.getEntryList();
    ROS_INFO("Modules found on network (Family | Name):");
    for (auto entry : *entry_list)
    {
      ROS_INFO("%s | %s", entry.family_.c_str(), entry.name_.c_str());
    }
  }

  /**
  * Get current HEBI actuator position offsets and bias future readings against them
  */
  void HebiBaseDiffDriveHardware::resetTravelOffset()
  {
    // Fill in feedback
    if (hebi_group_->getNextFeedback(*hebi_feedback_))
    {
      // Retrieve current positions
      hebi_positions_ = hebi_feedback_->getPosition();
      for (int i = 0; i < NUM_WHEELS; i++)
      {
        joints_[i].position_offset = hebi_positions_[i];
      }
    }
    else
    {
      ROS_ERROR("Could not get HEBI feedback data to calibrate travel offset");
    }
  }

  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void HebiBaseDiffDriveHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void HebiBaseDiffDriveHardware::updateJointsFromHardware()
  {
    // Fill in feedback
    bool feedback_received = false;
    if (hebi_group_->getNextFeedback(*hebi_feedback_))
    {
      feedback_received = true;
    } else {
      feedback_received = false;
    }

    if (feedback_received)
    {
      hebi_positions_ = hebi_feedback_->getPosition();
      for (int i = 0; i < NUM_WHEELS; i++)
      {
        double delta = hebi_positions_[i] - joints_[i].position - joints_[i].position_offset;

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0)
        {
          joints_[i].position += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
          ROS_DEBUG("Dropping overflow measurement from encoder");
        }
      }
    }
;
    if (feedback_received)
    {
      hebi_velocities_ = hebi_feedback_->getVelocity();
      for (int i = 0; i < NUM_WHEELS; i++)
      {
        if (i % 2 == LEFT)
        {
          joints_[i].velocity = hebi_velocities_[i];  // FIXME: May need to flip sign if ros_control doesn't know about axis
        }
        else
        { // assume RIGHT
          joints_[i].velocity = hebi_velocities_[i];
        }
      }
    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void HebiBaseDiffDriveHardware::writeCommandsToHardware()
  {
    double diff_speed_left = joints_[LEFT].velocity_command;
    double diff_speed_right = -joints_[RIGHT].velocity_command;
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);
    for (int i = 0; i < NUM_WHEELS; i++)
    {
      if (i % 2 == LEFT)
      {
        cmd_vel_vector_[i] = diff_speed_left;
      }
      else
      { // assume RIGHT
        cmd_vel_vector_[i] = diff_speed_right;
      }
    }

    hebi_command_->setVelocity(cmd_vel_vector_);
    hebi_group_->sendCommand(*hebi_command_);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void HebiBaseDiffDriveHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Husky reports travel in metres, need radians for ros_control RobotHW
  */
  double HebiBaseDiffDriveHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2.0;
  }

  /**
  * RobotHW provides velocity command in rad/s, Husky needs m/s,
  */
  double HebiBaseDiffDriveHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2.0;
  }


}  // namespace hebi_base_diff_drive_hardware