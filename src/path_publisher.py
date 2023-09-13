#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Float32
import tf
import geometry_msgs.msg
import numpy as np

class Path_Publisher():
    def __init__(self, pub_topic, pose_sub_topic):
        self.filter_path_publisher_ = rospy.Publisher(pub_topic, Path, queue_size = 1)
        # Added a new subscriber that subscribes to the gazebo/rviz resetting node
        self.clear_path = rospy.Subscriber('/clear_path_msg', String, self.clearpath)
        self.filter_subscription_ = rospy.Subscriber(
            pose_sub_topic, #'/pf/viz/inferred_pose', #'/amcl_pose', /odom
            PoseStamped,
            self.filter_listener_callback)
        self.filter_subscription_  #prevent unused variable warning
        self.filter_path = Path()

        self.filter_last_called = 0

    def filter_listener_callback(self, msgin):
        # only add new msgs to the path msg at the allowed rate 
        now = rospy.get_time()
        if now - self.filter_last_called < 0.1:
            return
        self.filter_last_called = now 
        # rospy.loginfo("received filtered odometry")
        newpose = PoseStamped()
        self.filter_path.header = msgin.header
        newpose.header = msgin.header
        #newpose.header.frame_id = "base_link"
        newpose.pose = msgin.pose
        self.filter_path.poses.append(newpose)
        # rospy.loginfo("publishing filter path")
        self.filter_path_publisher_.publish(self.filter_path)

    # Function that clears the published pose messages when "clear" is received
    def clearpath(self,data):
        if data.data == "clear":
            self.filter_path.poses.clear()





class Ground_Truth_Path_Publisher():

    def __init__(self, pose_sub_topic):
        self.filter_path_publisher_ = rospy.Publisher("ground_truth_path", Path, queue_size = 1)
        
        self.clear_path = rospy.Subscriber('/clear_path_msg', String, self.clearpath)

        self.filter_subscription_ = rospy.Subscriber( pose_sub_topic, Odometry, self.filter_listener_callback)
        self.filter_subscription_ 
        self.filter_path = Path()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.filter_last_called = 0
        
    def filter_listener_callback(self, msgin):
        now = rospy.get_time()
        if now - self.filter_last_called < 0.1:
            return
        self.filter_last_called = now 
        
        newpose = Odometry()
        self.filter_path.header = msgin.header
        newpose.header = msgin.header
        #newpose.header.frame_id = "base_link"
        newpose.pose = msgin.pose.pose
        self.filter_path.poses.append(newpose)
        # rospy.loginfo("publishing filter path")
        self.filter_path_publisher_.publish(self.filter_path)

        current_time = rospy.Time.now()
        translation = (msgin.pose.pose.position.x, msgin.pose.pose.position.y, msgin.pose.pose.position.z)
        rotation = (msgin.pose.pose.orientation.x, msgin.pose.pose.orientation.y, msgin.pose.pose.orientation.z, msgin.pose.pose.orientation.w)
        self.tf_broadcaster.sendTransform(translation, rotation , current_time, "base_link" , "odom" )

    # Function that clears the published pose messages when "clear" is received
    def clearpath(self,data):
        if data.data == "clear":
            self.filter_path.poses.clear()

class Accuracy( ):

    def __init__(self, ground_truth_topic, pred_pose_topic ) :
        self.ground_truth_ = rospy.Subscriber( ground_truth_topic , Odometry, self.ground_truth_callback)
        self.pose_est = rospy.Subscriber( pred_pose_topic, PoseStamped, self.estimated_callback)
        self.error_publisher = rospy.Publisher("RMSE" , Float32 , queue_size=10)

        self.ground_truth_pose = None
        self.estimated_pose = None
        self.error_values = []
        self.window_size = 100

    def ground_truth_callback(self, msg):
        self.ground_truth_pose = msg.pose.pose
        self.calculate_error()
        
    def estimated_callback(self, msg):
        self.estimated_pose = msg.pose
        self.calculate_error()


    def calculate_error(self):
        if self.ground_truth_pose is None or self.estimated_pose is None:
            return

        # Calculate Euclidean distance error
        dx = self.ground_truth_pose.position.x - self.estimated_pose.position.x
        dy = self.ground_truth_pose.position.y - self.estimated_pose.position.y
        distance_error = np.sqrt(dx**2 + dy**2 )

        # Append error value to the list
        self.error_values.append(distance_error)

        # Apply moving average filter
        if len(self.error_values) > self.window_size:
            self.error_values = self.error_values[-self.window_size:]

        # Calculate and print the moving average error
        rmse = np.mean(self.error_values)
        #rospy.loginfo("Moving Average Error: %.4f", moving_avg_error)
        self.error_publisher.publish(rmse)


def main():
    
    rospy.init_node("path_publisher", anonymous = True)
    
    pub_topic = rospy.get_param('~pub_topic', '/path')
    pose_sub_topic = rospy.get_param('~sub_topic')
    ground_truth_available = rospy.get_param('~groud_truth_aval')
    ground_truth_topic = rospy.get_param('~groud_truth_topic')

    path_publisher = Path_Publisher(pub_topic, pose_sub_topic)

    if ground_truth_available:
        #rospy.Subscriber(ground_truth_topic , Odometry, odom_callback, tf_broadcaster) #jet2/vicon_odom
        #filter_path_publisher_ = rospy.Publisher(pub_topic, Path, queue_size = 1)
        GT_path_publisher = Ground_Truth_Path_Publisher(ground_truth_topic)
        accuracy_print = Accuracy(ground_truth_topic, pose_sub_topic)


    rospy.loginfo("Starting the Path Publisher")
    rospy.spin()

if __name__ == '__main__':
    main()