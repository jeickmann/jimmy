#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Quaternion
from tf.broadcaster import TransformBroadcaster
import tf.transformations

class MockLocalization():
    def __init__(self):
        rospy.init_node("mock_localization")
        nodename = rospy.get_name()

        self.rate = rospy.get_param("~rate", 20)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.poseCallback)
        self.pose = Pose()
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]
        self.broadcaster = TransformBroadcaster()

    def spin(self):
        r = rospy.Rate(self.rate)
        
        while(not rospy.is_shutdown()):
            self.broadcaster.sendTransform(
                (self.pose.position.x, self.pose.position.y, self.pose.position.z),
                (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w),
                rospy.Time.now(),
                'odom',
                'map'
                )
            r.sleep()


    def poseCallback(self,msg):
        self.pose = msg.pose.pose

if __name__ == '__main__':
    mockLocalization = MockLocalization()
    mockLocalization.spin()