#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class Encoder():
    def __init__(self): 
        rospy.init_node("encoder")
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.wheel_mult = 0
        self.prev_encoder = 0
        self.vel = 0
        
        self.timeout_ticks = rospy.get_param('~timeout_ticks',4)
        self.ticks_per_meter = rospy.get_param('ticks_meter', 20)
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.001)
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.rate = rospy.get_param('~rate',100)

        rospy.Subscriber("ticks", Int16, self.ticksCallback) 
        rospy.Subscriber("wheel_cmd", Float64, self.cmdCallback)
        
        self.pub_vel = rospy.Publisher('wheel_vel', Float64, queue_size=10)

        self.pub_cmd = rospy.Publisher('wheel_cmd32', Float32, queue_size=10)
        pass
    
    def spin(self):
        self.r = rospy.Rate(self.rate) 
        self.then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = rospy.Time.now()
        while not rospy.is_shutdown():
            self.calcVelocity()
            self.r.sleep()

    def calcVelocity(self):
        self.dt_duration = rospy.Time.now() - self.then
        self.dt = self.dt_duration.to_sec()
        
        if (self.wheel_latest == self.wheel_prev):
            # we haven't received an updated wheel lately
            cur_vel = (1.0 / self.ticks_per_meter) / self.dt    # if we got a tick right now, this would be the velocity
            if abs(cur_vel) < self.vel_threshold: 
                # if the velocity is < threshold, consider our velocity 0
                self.vel = 0
            else:
                if abs(cur_vel) < self.vel:
                    self.vel = cur_vel
            
        else:
            # we received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            self.vel = cur_vel
            self.wheel_prev = self.wheel_latest
            self.then = rospy.Time.now()
            
        self.pub_vel.publish(self.vel)

    def ticksCallback(self, msg):
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
           
         
        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
        self.prev_encoder = enc

    def cmdCallback(self, msg):
        cmd = msg.data

        self.pub_cmd.publish(cmd)

if __name__ == '__main__':
    encoder = Encoder()
    encoder.spin()