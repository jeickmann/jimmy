#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DiffDrive():
    def __init__(self):
        rospy.init_node("diff_drive")
        nodename = rospy.get_name()
    
        self.w = rospy.get_param("~base_width", 0.2)
    
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout = rospy.get_param("~timeout", 1)
        self.left = 0
        self.right = 0
        self.dx = 0
        self.dr = 0
        self.dy = 0

    def spin(self):
        r = rospy.Rate(self.rate)
        self.last_target = rospy.Time.now()
        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()

    def spinOnce(self):
        dT = rospy.Time.now() - self.last_target
        if(dT.to_sec() > self.timeout):
            self.dx = 0
            self.dr = 0
        
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
            
    def twistCallback(self,msg):
        self.last_target = rospy.Time.now() 
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

if __name__ == '__main__':
    """ main """
    diffDrive = DiffDrive()
    diffDrive.spin()