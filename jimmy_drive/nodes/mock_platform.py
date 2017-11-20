#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int32

from geometry_msgs.msg import Pose

class MockPlattform():
    def __init__(self):
        rospy.init_node("mock_plattform")
        nodename = rospy.get_name()
    
        self.lvel = 0
        self.rvel = 0

        self.lticks = 0
        self.rticks = 0

        self.pub_lvel = rospy.Publisher('lwheel_vel', Float32, queue_size=10)
        self.pub_rvel = rospy.Publisher('rwheel_vel', Float32, queue_size=10)

        self.pub_lticks = rospy.Publisher('lticks', Int32, queue_size=10)
        self.pub_rticks = rospy.Publisher('rticks', Int32, queue_size=10)
        rospy.Subscriber('lwheel_vtarget', Float32, self.lCmdCallback)
        rospy.Subscriber('rwheel_vtarget', Float32, self.rCmdCallback)
        
        self.ticks_meter = rospy.get_param("~ticks_meter", 1130)
        self.rate = rospy.get_param("~rate", 50)
        self.timeout = rospy.get_param("~timeout", 0.1)
        
    def spin(self):
        r = rospy.Rate(self.rate)
        self.lastSpin = rospy.Time.now()
        self.last_cmd = rospy.Time.now()
        
        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()

    def spinOnce(self):
        dTLastCmd = rospy.Time.now() - self.last_cmd
        if(dTLastCmd.to_sec() > self.timeout):
            self.lvel = 0
            self.rvel = 0
        
        dT = rospy.Time.now() - self.lastSpin
        dT_sec = dT.to_sec()
        self.lastSpin = rospy.Time.now()

        self.lticks += self.lvel * dT_sec * self.ticks_meter
        self.rticks += self.rvel * dT_sec * self.ticks_meter
        
        self.pub_rvel.publish(self.rvel)
        self.pub_lvel.publish(self.lvel)
        self.pub_rticks.publish(self.rticks)
        self.pub_lticks.publish(self.lticks)
            
    def lCmdCallback(self,msg):
        self.last_cmd = rospy.Time.now() 
        self.lvel = msg.data

    def rCmdCallback(self,msg):
        self.last_cmd = rospy.Time.now() 
        self.rvel = msg.data

if __name__ == '__main__':
    """ main """
    mockPlattform = MockPlattform()
    mockPlattform.spin()