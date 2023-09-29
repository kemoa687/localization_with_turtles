#include <ros/ros.h>
#include <turtlesim/Pose.h>

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
  // Create a new publisher for the transformed pose message
  static ros::NodeHandle nh;
  static ros::Publisher transformed_pose_pub = nh.advertise<turtlesim::Pose>("transformed_pose", 10);

  // Publish the received pose message on the "transformed_pose" topic
  transformed_pose_pub.publish(*msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_transformer");
  ros::NodeHandle nh;

  // Create a subscriber for the "turtle4/pose" topic
  ros::Subscriber pose_sub = nh.subscribe("turtle4/pose", 10, poseCallback);

  ros::spin();

  return 0;
}
