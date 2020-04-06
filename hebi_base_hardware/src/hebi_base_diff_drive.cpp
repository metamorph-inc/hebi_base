// Based heavily on https://github.com/husky/husky/blob/melodic-devel/husky_base/src/husky_base.cpp
// and https://github.com/HebiRobotics/hebi-cpp-examples/blob/master/kits/rosie/rosie_demo.cpp


#include "hebi_base_hardware/hebi_base_diff_drive_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hebi_base_diff_drive");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // Initialize robot hardware and link to controller manager
  hebi_base_diff_drive_hardware::HebiBaseDiffDriveHardware hebi_base(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&hebi_base, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate r(200); // 200 hz
  ros::Duration elapsed_duration;
  ros::Time t_prev = ros::Time::now();
  while (ros::ok())
  {
    ros::Time t = ros::Time::now();
    elapsed_duration = t - t_prev;
    t_prev = t;
    // Process control loop
    hebi_base.updateJointsFromHardware();
    cm.update(ros::Time::now(), elapsed_duration);
    hebi_base.writeCommandsToHardware();

    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
