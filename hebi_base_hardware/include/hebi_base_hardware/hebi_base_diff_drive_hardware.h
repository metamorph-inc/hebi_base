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

      std::string eigenToString(const Eigen::MatrixXd& mat);

      double linearToAngular(const double &travel) const;

      double angularToLinear(const double &angle) const;

      void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

      void feedbackCallback(const sensor_msgs::JointState::ConstPtr& data);

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
      std::string hebi_group_name_;

      // HEBI Feedback
      ros::Subscriber feedback_subscriber_;
      bool feedback_received_;
      bool offset_calculated_;
      std::vector<double> fbk_positions_;
      std::vector<double> fbk_velocities_;

      // HEBI Command
      ros::Publisher hebi_publisher_;
      std::vector<double> cmd_velocities_;
      sensor_msgs::JointState cmd_msg_;

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