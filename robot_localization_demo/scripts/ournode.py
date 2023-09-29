import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

def pose_publisher():
    rospy.init_node('purplish_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/purplish_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_link'  # Replace with your frame ID

        # Set the position and orientation of the pose
        pose_msg.pose.position.x = 1.0  # Replace with your desired position
        pose_msg.pose.position.y = 2.0  # Replace with your desired position
        pose_msg.pose.position.z = 0.5  # Replace with your desired position

        # Set the orientation as a purplish color (quaternion)
        quat = quaternion_from_euler(0.7, 0.0, 0.7)  # Replace with your desired orientation
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        pub.publish(pose_msg)
        rate.sleep()

if __name__ == 'main':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass