#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from tf2_geometry_msgs import do_transform_pose
import tf2_py as tf2
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node("transformation_visualization_node")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Spawn a new turtle and store its name.
    rospy.wait_for_service("spawn")
    spawn_visualization_turtle = Spawn()
    spawn_visualization_turtle.request.x = 0
    spawn_visualization_turtle.request.y = 0
    spawn_visualization_turtle.request.theta = 0
    client_spawn = rospy.ServiceProxy("spawn", Spawn)
    response = client_spawn(spawn_visualization_turtle)
    visualization_turtle_name = response.name

    # Set pen color to light green.
    configure_visualization_turtle = SetPen()
    configure_visualization_turtle.request.r = 0
    configure_visualization_turtle.request.g = 255
    configure_visualization_turtle.request.b = 0
    configure_visualization_turtle.request.width = 3
    configure_visualization_turtle.request.off = 0
    client_configure = rospy.ServiceProxy(
        visualization_turtle_name + "/set_pen", SetPen
    )
    client_configure(configure_visualization_turtle)

    # Log message.
    rospy.loginfo("Absolute position estimate visualized by '%s' using a green pen.", visualization_turtle_name)

    # Visualize the estimated position of the turtle in the map frame.
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            base_link_to_map_transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
        except tf2.LookupException as ex:
            rospy.loginfo("tf2_ros.Buffer.lookup_transform failed: %s", str(ex))
            rospy.sleep(1.0)
            continue

        # Move visualization turtle to the estimated position.
        pose_base_link = PoseStamped()
        pose_base_link.header.stamp = rospy.Time.now()
        pose_base_link.header.frame_id = "base_link"
        pose_base_link.pose.position.x = 0.0
        pose_base_link.pose.position.y = 0.0
        pose_base_link.pose.position.z = 0.0
        pose_base_link.pose.orientation.x = 0.0
        pose_base_link.pose.orientation.y = 0.0
        pose_base_link.pose.orientation.z = 0.0
        pose_base_link.pose.orientation.w = 1.0

        pose_map = do_transform_pose(pose_base_link, base_link_to_map_transform)
        
        quaternion = quaternion_from_euler(0.0, 0.0, pose_map.pose.orientation.z)
        
        visualize_current_pose = TeleportAbsolute()
        visualize_current_pose.request.x = pose_map.pose.position.x
        visualize_current_pose.request.y = pose_map.pose.position.y
        visualize_current_pose.request.theta = quaternion[2]

        client = rospy.ServiceProxy(
            visualization_turtle_name + "/teleport_absolute", TeleportAbsolute
        )
        client(visualize_current_pose)

        # Sleep until the next update.
        rate.sleep()

if __name__ == "__main__":
    main()
