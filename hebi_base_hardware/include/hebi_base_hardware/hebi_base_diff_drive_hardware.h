// Based heavily on https://github.com/husky/husky/blob/melodic-devel/husky_base/include/husky_base/husky_hardware.h

#ifndef HEBI_BASE_DIFF_DRIVE_HARDWARE_H
#define HEBI_BASE_DIFF_DRIVE_HARDWARE_H

#include <string>
#include "Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"

namespace hebi_base_diff_drive_hardware
{

  /**
  * Class representing HEBI Base Diff Drive hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class HebiBaseDiffDriveHardware: public hardware_interface::RobotHW
  {
    public:
      HebiBaseDiffDriveHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

      void updateJointsFromHardware();

      void writeCommandsToHardware();
      
    private:
    
      void resetTravelOffset();

      void registerControlInterfaces();

      std::vector<std::string> getHebiFamilyNameFromParam(const std::string &param_name, const std::string &param_default_value);

      void printHebiLookup(hebi::Lookup &hebi_lookup);

      std::string eigenToString(const Eigen::MatrixXd& mat);

      double linearToAngular(const double &travel) const;

      double angularToLinear(const double &angle) const;

      void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

      ros::NodeHandle nh_, private_nh_;

      // ROS Control interfaces
      hardware_interface::JointStateInterface joint_state_interface_;
      hardware_interface::VelocityJointInterface velocity_joint_interface_;

      // Diagnostics
      // TODO

      // ROS Parameters
      double wheel_diameter_, max_accel_, max_speed_;

      double polling_timeout_;

      // HEBI
      std::shared_ptr<hebi::Group> hebi_group_;

      // HEBI Feedback
      std::shared_ptr<hebi::GroupFeedback> hebi_feedback_;
      // hebi::GroupFeedback hebi_feedback_;
      Eigen::VectorXd hebi_positions_;
      Eigen::VectorXd hebi_velocities_;

      // HEBI Command
      std::shared_ptr<hebi::GroupCommand> hebi_command_;
      // hebi::GroupCommand hebi_command_;
      Eigen::VectorXd cmd_vel_vector_;

      /**
      * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
      */
      struct Joint
      {
        double position;
        double position_offset;
        double velocity;
        double effort;
        double velocity_command;

        Joint() :
          position(0), position_offset(0), velocity(0), effort(0), velocity_command(0)
        { }
      } joints_[4];
  };

}  // hebi_base_diff_drive_hardware
#endif  // HEBI_BASE_DIFF_DRIVE_HARDWARE_H