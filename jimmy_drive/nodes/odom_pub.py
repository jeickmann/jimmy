#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class OdomPub():
    def __init__(self):
        rospy.init_node("odom_pub")
        nodename = rospy.get_name()
    
        self.base_width = rospy.get_param("~base_width", 0.11)
        self.ticks_meter = rospy.get_param("~ticks_meter", 1130)
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.rate = rospy.get_param("~rate", 20)
    
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.Subscriber('lticks', Int32, self.lticksCallback)
        rospy.Subscriber('rticks', Int32, self.rticksCallback)
            
        self.lticks = 0
        self.rticks = 0
        self.lastticks_l = 0
        self.lastticks_r = 0
        self.x = 0
        self.y = 0
        self.th = 0

    def spin(self):
        r = rospy.Rate(self.rate)
        self.last = rospy.Time.now()

        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()

    def spinOnce(self):
        now = rospy.Time.now()
        dT = now - self.last
        dT = dT.to_sec()
        self.last = now
        
        d_left = float(self.lticks - self.lastticks_l) / self.ticks_meter
        d_right = float(self.rticks - self.lastticks_r) / self.ticks_meter
        self.lastticks_l = self.lticks
        self.lastticks_r = self.rticks

        d = (d_left + d_right)/2
        th = ( d_right - d_left ) / self.base_width

        self.dx = d / dT
        self.dr = th / dT

        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin( self.th / 2 )
        quaternion.w = cos( self.th / 2 )
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            now,
            self.base_frame_id,
            self.odom_frame_id
            )
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

    def lticksCallback(self,msg):
        self.lticks = msg.data
        pass
    
    def rticksCallback(self, msg):
        self.rticks = msg.data
        pass

if __name__ == '__main__':
    """ main """
    odomPub = OdomPub()
    odomPub.spin()