#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R


# ROS message types
from nav_msgs.msg import Odometry

SUBSCRIBER_TOPIC_NAME = "target/filtered_twist"
PUBLISHER_TOPIC_NAME = "target/estimated_twist"
FREQ = 100

class ObjectRepublisher():

    def __init__(self):

        # Subscribe target object estimated callback
        rospy.Subscriber(SUBSCRIBER_TOPIC_NAME, Odometry, self.read2publish)

        # Setup ros publishers
        self.target_state_publisher = rospy.Publisher(PUBLISHER_TOPIC_NAME, Odometry, queue_size=1)

        #  info about the node
        self.name = "Object republish estimated Kalman state"
        self.msg_index = 0


    def read2publish(self, msg):



        # geometry_msgs/PoseWithCovarianceStamped
        pub_msg = Odometry()
        # header
        pub_msg.header.stamp = msg.header.stamp
        pub_msg.header.seq = msg.header.seq
        pub_msg.header.frame_id = msg.header.frame_id
        # pose
        pub_msg.pose.pose = msg.pose.pose
        # pub_msg.pose.pose.position.x = msg.pose.pose.position.x
        # pub_msg.pose.pose.position.y = msg.pose.pose.position.y
        # pub_msg.pose.pose.position.z = msg.pose.pose.position.z
        pub_msg.pose.covariance = msg.pose.covariance
        # pose.pose.orientation
        # pub_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
        # pub_msg.pose.pose.orientation.y = target_EE_orientation[1]
        # pub_msg.pose.pose.orientation.z = target_EE_orientation[2]
        # pub_msg.pose.pose.orientation.w = target_EE_orientation[3]
        # twist
        pub_msg.twist.twist.linear = msg.twist.twist.linear
        # compute wmega from euler derivatives
        quat = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        transfer_func = self.transfunc4Euler2Wmega(quat.as_euler('zxy'))
        wmega = np.dot(transfer_func, np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]))

        pub_msg.twist.twist.angular.x = wmega[0]
        pub_msg.twist.twist.angular.y = wmega[1]
        pub_msg.twist.twist.angular.z = wmega[2]
        pub_msg.twist.covariance = msg.twist.covariance

        self.target_state_publisher.publish(pub_msg)


    def transfunc4Euler2Wmega(self, param):
        '''
        transfer function: from euler angle rate to angular velocity w
        :param param: euler angles
        :return: transfer function
        '''
        transfunceul2w = np.zeros((3,3))
        transfunceul2w[0, 0] = 0; transfunceul2w[0, 1] = -np.sin(param[0]); transfunceul2w[0, 2] = np.cos(param[0])*np.cos(param[1])
        transfunceul2w[1, 0] = 0; transfunceul2w[1, 1] = np.cos(param[0]); transfunceul2w[1, 2] = np.sin(param[0])*np.cos(param[1])
        transfunceul2w[2, 0] = 1; transfunceul2w[2, 1] = 0;  transfunceul2w[2, 2] = -np.sin(param[1])

        return transfunceul2w

if __name__=="__main__":

    try:
        # Initialize node
        rospy.init_node("ros_republish_object_estimated_state", anonymous=True)
        # Initialize node class
        ObjectRepublisher = ObjectRepublisher()

        rospy.loginfo("%s: Setup target reader.", ObjectRepublisher.name)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

 # end
