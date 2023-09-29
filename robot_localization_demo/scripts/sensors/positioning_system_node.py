#!/usr/bin/env python

import rospy
import argparse
# from positioning_system import TurtlePositioningSystem
import rospy
import random
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from tf.transformations import quaternion_from_euler

class TurtlePositioningSystem:
    def __init__(self, node_handle, frequency,
                 error_x_systematic, error_x_random, error_y_systematic, error_y_random,
                 error_yaw_systematic, error_yaw_random, visualize):
        self.node_handle = node_handle
        self.turtle_pose_subscriber = rospy.Subscriber("turtle1/pose", Pose, self.turtle_pose_callback)
        self.turtle_pose_publisher = rospy.Publisher("turtle1/sensors/pose", PoseWithCovarianceStamped, queue_size=16)
        self.frequency = frequency
        self.random_generator = random.Random()
        self.random_distribution_x = (error_x_systematic, error_x_random)
        self.random_distribution_y = (error_y_systematic, error_y_random)
        self.random_distribution_yaw = (error_yaw_systematic, error_yaw_random)
        self.frame_sequence = 0
        self.visualize = visualize
        self.visualization_turtle_name = ""
        self.cached_pose = Pose()  

    def spin(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            # Distort real pose to get a 'measurement'.
            measurement = self.cached_pose
            measurement.x += random.uniform(*self.random_distribution_x)
            measurement.y += random.uniform(*self.random_distribution_y)
            measurement.theta += random.uniform(*self.random_distribution_yaw)

            # Publish measurement.
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.seq = self.frame_sequence + 1
            current_pose.header.stamp = rospy.Time.now()
            current_pose.header.frame_id = "map"
            current_pose.pose.pose.position.x = measurement.x
            current_pose.pose.pose.position.y = measurement.y
            current_pose.pose.pose.position.z = 0.0
            q = quaternion_from_euler(0.0, 0.0, measurement.theta)
            current_pose.pose.pose.orientation.x = q[0]
            current_pose.pose.pose.orientation.y = q[1]
            current_pose.pose.pose.orientation.z = q[2]
            current_pose.pose.pose.orientation.w = q[3]

            # Publish the covariance (assuming similar structure to C++ code)
            current_pose.pose.covariance = [
                (self.random_distribution_x[0] + self.random_distribution_x[1]) ** 2, 0, 0, 0, 0, 0,
                0, (self.random_distribution_y[0] + self.random_distribution_y[1]) ** 2, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, (self.random_distribution_yaw[0] + self.random_distribution_yaw[1]) ** 2]

            self.turtle_pose_publisher.publish(current_pose)

            if self.is_visualization_requested() and self.is_visualization_turtle_available():
                self.move_visualization_turtle(measurement)

            rate.sleep()

    def turtle_pose_callback(self, message):
        self.cached_pose_timestamp = rospy.Time.now()
        self.cached_pose = message

        # If this is the first message, initialize the visualization turtle.
        if self.is_visualization_requested() and not self.is_visualization_turtle_available():
            self.spawn_and_configure_visualization_turtle(message)

    def spawn_and_configure_visualization_turtle(self, initial_pose):
        if self.is_visualization_requested() and not self.is_visualization_turtle_available():
            # Spawn a new turtle and store its name.
            rospy.wait_for_service("spawn")
            try:
                spawn_visualization_turtle = rospy.ServiceProxy("spawn", Spawn)
                response = spawn_visualization_turtle(initial_pose.x, initial_pose.y, initial_pose.theta)
                self.visualization_turtle_name = response.name

                # Set pen color to blue.
                configure_visualization_turtle = rospy.ServiceProxy(
                    self.visualization_turtle_name + "/set_pen", SetPen
                )
                configure_visualization_turtle(0, 0, 255, 1, 0)

                # Log message.
                rospy.loginfo("Absolute position measurement visualized by '%s' using a blue pen.",
                              self.visualization_turtle_name)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def move_visualization_turtle(self, measurement):
        if self.is_visualization_requested() and self.is_visualization_turtle_available():
            # Move visualization turtle to the 'measured' position.
            try:
                visualize_current_pose = rospy.ServiceProxy(
                    self.visualization_turtle_name + "/teleport_absolute", TeleportAbsolute
                )
                visualize_current_pose(measurement.x, measurement.y, measurement.theta)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def is_visualization_requested(self):
        return self.visualize

    def is_visualization_turtle_available(self):
        return self.visualization_turtle_name != ""

if __name__ == "__main__":
    rospy.init_node("turtle_positioning_system")
    positioning_system = TurtlePositioningSystem(
        rospy.get_rostime(),
        frequency=10.0,  # Adjust the frequency as needed
        error_x_systematic=0.0,
        error_x_random=0.1,
        error_y_systematic=0.0,
        error_y_random=0.1,
        error_yaw_systematic=0.0,
        error_yaw_random=0.1,
        visualize=True,
    )
    positioning_system.spin()
def main():
    parser = argparse.ArgumentParser(description="Positioning System")
    parser.add_argument("--frequency", "-f", type=float, default=1.0, help="Set measurement frequency (Hz)")
    parser.add_argument("--error-x-systematic", "-X", type=float, default=0.0, help="Set systematic error on X")
    parser.add_argument("--error-x-random", "-x", type=float, default=0.0, help="Set random error on X")
    parser.add_argument("--error-y-systematic", "-Y", type=float, default=0.0, help="Set systematic error on Y")
    parser.add_argument("--error-y-random", "-y", type=float, default=0.0, help="Set random error on Y")
    parser.add_argument("--error-yaw-systematic", "-T", type=float, default=0.0, help="Set systematic error on yaw")
    parser.add_argument("--error-yaw-random", "-t", type=float, default=0.0, help="Set random error on yaw")
    parser.add_argument("--visualize", "-v", action="store_true", help="Visualize positioning system measurement")

    args = parser.parse_args()

    rospy.init_node("turtle_positioning_system")
    node_handle = rospy.get_rostime()

    turtle_positioning_system = TurtlePositioningSystem(
        node_handle,
        frequency=args.frequency,
        error_x_systematic=args.error_x_systematic,
        error_x_random=args.error_x_random,
        error_y_systematic=args.error_y_systematic,
        error_y_random=args.error_y_random,
        error_yaw_systematic=args.error_yaw_systematic,
        error_yaw_random=args.error_yaw_random,
        visualize=args.visualize,
    )

    turtle_positioning_system.spin()

if __name__ == "__main__":
    main()
