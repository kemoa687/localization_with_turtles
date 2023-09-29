#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>

int main(int argc, char** argv) {
  float x = 1;
  float y = 1;
  float theta = 0.5 ;
  ros::init(argc, argv, "pose_publisher");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<turtlesim::Pose>("poseee", 10);
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    turtlesim::Pose pose_msg;
    // Set the pose values
    pose_msg.x = x ;
    pose_msg.y = y ;
    pose_msg.theta = 0.1;
    pose_msg.linear_velocity = 0.5;
    pose_msg.angular_velocity = 0.0;

    x=x+0.2 ;
    y=y+0.2 ;
    // theta = theta + 0.001;
    // Publish the pose message
    pose_pub.publish(pose_msg);

    ros::spinOnce();
    loop_rate.sleep(); 
  }

  return 0;
}