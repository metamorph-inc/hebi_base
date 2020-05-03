// Based heavily on https://github.com/husky/husky/blob/melodic-devel/husky_base/src/husky_hardware.cpp

#include "hebi_base_hardware/hebi_base_diff_drive_hardware.h"
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp> 
#include <thread>
#include "Eigen/Dense"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SendCommandWithAcknowledgementSrv.h"
#include "hebiros/CommandMsg.h"

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
    private_nh_(private_nh),
    feedback_received_(false),
    offset_calculated_(false),
    cmd_velocities_(NUM_WHEELS)
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

    hebi_group_name_ = "hebi_base";

    //Create a client which uses the service to create a group
    ros::ServiceClient add_group_client = nh_.serviceClient<hebiros::AddGroupFromNamesSrv>(
      "/hebiros/add_group_from_names");

    ros::ServiceClient send_command_with_acknowledgement = nh_.serviceClient<hebiros::SendCommandWithAcknowledgementSrv>(
      "/hebiros/"+hebi_group_name_+"/send_command_with_acknowledgement");

    //Create a subscriber to receive feedback from a group
    //Register feedback_callback as a callback which runs when feedback is received
    feedback_subscriber_ = nh_.subscribe<sensor_msgs::JointState>(
      "/hebiros/"+hebi_group_name_+"/feedback/joint_state", 100, &HebiBaseDiffDriveHardware::feedbackCallback, this);

    //Create a publisher to send desired commands to a group
    hebi_publisher_ = nh_.advertise<sensor_msgs::JointState>(
      "/hebiros/"+hebi_group_name_+"/command/joint_state", 100);

    cmd_msg_.name = hebi_mapping;

    hebiros::AddGroupFromNamesSrv add_group_srv;
    add_group_srv.request.group_name = hebi_group_name_;
    add_group_srv.request.families = hebi_families;
    add_group_srv.request.names = hebi_names;
    while(!add_group_client.call(add_group_srv)) {}

    hebiros::SendCommandWithAcknowledgementSrv send_cmd_w_ack_srv;
    send_cmd_w_ack_srv.request.command.name = hebi_mapping;
    send_cmd_w_ack_srv.request.command.settings.name = hebi_mapping;
    send_cmd_w_ack_srv.request.command.settings.control_strategy = {4, 4, 4, 4};
    send_cmd_w_ack_srv.request.command.settings.position_gains.kp = {3, 3, 3, 3};
    send_cmd_w_ack_srv.request.command.settings.velocity_gains.kp = {0.3, 0.3, 0.3, 0.3};
    send_cmd_w_ack_srv.request.command.settings.velocity_gains.feed_forward = {1, 1, 1, 1};
    send_cmd_w_ack_srv.request.command.settings.velocity_gains.max_target = {9.617128, 9.617128, 9.617128, 9.617128};
    send_cmd_w_ack_srv.request.command.settings.velocity_gains.min_target = {-9.617128, -9.617128, -9.617128, -9.617128};
    send_command_with_acknowledgement.call(send_cmd_w_ack_srv);

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
  * Get current HEBI actuator position offsets and bias future readings against them
  */
  void HebiBaseDiffDriveHardware::resetTravelOffset()
  {
    // Retrieve current positions
    for (int i = 0; i < NUM_WHEELS; i++)
    {
      joints_[i].position_offset = fbk_positions_[i];
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
    if (feedback_received_)
    {
      if (!offset_calculated_)
      {
        resetTravelOffset();
        offset_calculated_ = true;
      }

      for (int i = 0; i < NUM_WHEELS; i++)
      {
        double delta = fbk_positions_[i] - joints_[i].position - joints_[i].position_offset;

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

    if (feedback_received_)
    {
      for (int i = 0; i < NUM_WHEELS; i++)
      {
        if (i % 2 == LEFT)
        {
          joints_[i].velocity = fbk_velocities_[i];  // FIXME: May need to flip sign if ros_control doesn't know about axis
        }
        else
        { // assume RIGHT
          joints_[i].velocity = fbk_velocities_[i];
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
        cmd_velocities_[i] = diff_speed_left;
      }
      else
      { // assume RIGHT
        cmd_velocities_[i] = diff_speed_right;
      }
    }
    cmd_msg_.velocity = cmd_velocities_;
    hebi_publisher_.publish(cmd_msg_);
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

  /**
  * Callback
  */
  void HebiBaseDiffDriveHardware::feedbackCallback(const sensor_msgs::JointState::ConstPtr& data) {
    fbk_positions_ = data->position;
    fbk_velocities_ = data->velocity;
    feedback_received_ = true;
  }

}  // namespace hebi_base_diff_drive_hardware