#!/usr/bin/env python

import rospy
import argparse
# from odometry import TurtleOdometry
#!/usr/bin/env python

import rospy
import argparse
import random
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen, TeleportRelative
from tf.transformations import quaternion_from_euler

class TurtleOdometry:
    def __init__(self, node_handle, frequency,
                 error_vx_systematic, error_vx_random, error_wz_systematic, error_wz_random, visualize):
        self.node_handle = node_handle
        self.turtle_pose_subscriber = rospy.Subscriber("turtle1/pose", Pose, self.turtle_pose_callback)
        self.turtle_twist_publisher = rospy.Publisher("turtle1/sensors/twist", TwistWithCovarianceStamped, queue_size=16)
        self.frequency = frequency
        self.random_generator = random.Random()
        self.random_distribution_vx = (error_vx_systematic, error_vx_random)
        self.random_distribution_wz = (error_wz_systematic, error_wz_random)
        self.frame_sequence = 0
        self.visualize = visualize
        self.visualization_turtle_name = ""
        self.cached_pose = Pose()

    def spin(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            # Distort real twist to get a 'measurement'.
            measurement = self.cached_pose
            measurement.linear_velocity *= (1.0 + random.uniform(*self.random_distribution_vx))
            measurement.angular_velocity += measurement.linear_velocity * random.uniform(*self.random_distribution_wz)
            # Publish measurement.
            current_twist = TwistWithCovarianceStamped()
            current_twist.header.seq = self.frame_sequence + 1
            current_twist.header.stamp = rospy.Time.now()
            current_twist.header.frame_id = "base_link"
            current_twist.twist.twist.linear.x = measurement.linear_velocity
            current_twist.twist.twist.linear.y = 0.0
            current_twist.twist.twist.linear.z = 0.0
            current_twist.twist.twist.angular.x = 0.0
            current_twist.twist.twist.angular.y = 0.0
            current_twist.twist.twist.angular.z = measurement.angular_velocity
            # Assuming a similar covariance structure as in the C++ code
            current_twist.twist.covariance = [
                (self.random_distribution_vx[0] + self.random_distribution_vx[1]) ** 2, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, (measurement.linear_velocity * (self.random_distribution_wz[0] + self.random_distribution_wz[1])) ** 2
            ]
            self.turtle_twist_publisher.publish(current_twist)

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

                # Set pen color to red.
                configure_visualization_turtle = rospy.ServiceProxy(
                    self.visualization_turtle_name + "/set_pen", SetPen
                )
                configure_visualization_turtle(255, 0, 0, 1, 0)

                # Log message.
                rospy.loginfo("Relative position measurement (odometry) visualized by '%s' with a red pen.",
                              self.visualization_turtle_name)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def move_visualization_turtle(self, measurement):
        if self.is_visualization_requested() and self.is_visualization_turtle_available():
            # Move visualization turtle to the 'measured' position.
            try:
                visualize_current_twist = rospy.ServiceProxy(
                    self.visualization_turtle_name + "/teleport_relative", TeleportRelative
                )
                visualize_current_twist(measurement.linear_velocity / self.frequency, measurement.angular_velocity / self.frequency)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def is_visualization_requested(self):
        return self.visualize

    def is_visualization_turtle_available(self):
        return self.visualization_turtle_name != ""

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Turtle Odometry")
    parser.add_argument("--frequency", "-f", type=float, default=1.0, help="Set measurement frequency (Hz)")
    parser.add_argument("--error-vx-systematic", "-X", type=float, default=0.0, help="Set systematic error on linear velocity")
    parser.add_argument("--error-vx-random", "-x", type=float, default=0.0, help="Set random error on linear velocity")
    parser.add_argument("--error-wz-systematic", "-W", type=float, default=0.0, help="Set systematic error on angular velocity")
    parser.add_argument("--error-wz-random", "-w", type=float, default=0.0, help="Set random error on angular velocity")
    parser.add_argument("--visualize", "-v", action="store_true", help="Visualize odometry measurement")

    args = parser.parse_args()

    rospy.init_node("turtle_odometry")
    node_handle = rospy.get_rostime()

    turtle_odometry = TurtleOdometry(
        node_handle,
        frequency=args.frequency,
        error_vx_systematic=args.error_vx_systematic,
        error_vx_random=args.error_vx_random,
        error_wz_systematic=args.error_wz_systematic,
        error_wz_random=args.error_wz_random,
        visualize=args.visualize,
    )

    turtle_odometry.spin()


def main():
    parser = argparse.ArgumentParser(description="Turtle Odometry")
    parser.add_argument("--frequency", "-f", type=float, default=1.0, help="Set measurement frequency (Hz)")
    parser.add_argument("--error-vx-systematic", "-X", type=float, default=0.0, help="Set systematic error on X velocity")
    parser.add_argument("--error-vx-random", "-x", type=float, default=0.0, help="Set random error on X velocity")
    parser.add_argument("--error-wz-systematic", "-T", type=float, default=0.0, help="Set systematic error on angular velocity")
    parser.add_argument("--error-wz-random", "-t", type=float, default=0.0, help="Set random error on angular velocity")
    parser.add_argument("--visualize", "-v", action="store_true", help="Visualize positioning system measurement")

    args = parser.parse_args()

    rospy.init_node("turtle_odometry")
    node_handle = rospy.get_rostime()

    turtle_odometry = TurtleOdometry(
        node_handle,
        frequency=args.frequency,
        error_vx_systematic=args.error_vx_systematic,
        error_vx_random=args.error_vx_random,
        error_wz_systematic=args.error_wz_systematic,
        error_wz_random=args.error_wz_random,
        visualize=args.visualize,
    )

    turtle_odometry.spin()

if __name__ == "__main__":
    main()