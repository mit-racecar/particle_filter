#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class PoseToOdometryNode:
    def __init__(self):
        rospy.init_node('pose_to_odometry_node', anonymous=True)
        self.pose_sub = rospy.Subscriber('pose2D', Pose2D, self.pose_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odom_msg = Odometry()        
        self.odom_msg.header.frame_id = 'map'
        self.odom_msg.child_frame_id = 'base_link'

    def pose_callback(self, pose_msg):        
        current_time = rospy.Time.now()
        self.odom_msg.header.stamp = current_time
        self.odom_msg.pose.pose.position.x = pose_msg.x
        self.odom_msg.pose.pose.position.y = pose_msg.y        
        self.odom_msg.pose.pose.position.z = 0.0
        
        quaternion = Quaternion(*quaternion_from_euler(0, 0, pose_msg.theta))
        
        self.odom_msg.pose.pose.orientation = quaternion
        self.odom_msg.twist.twist.linear.x = 0.0        
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0        
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(self.odom_msg)

if __name__ == '__main__':    
    try:
        node = PoseToOdometryNode()        
        rospy.spin()
    except rospy.ROSInterruptException:        
        pass