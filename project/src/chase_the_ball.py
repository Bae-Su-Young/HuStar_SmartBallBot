#!/usr/bin/env python

import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

K_LAT_DIST_TO_STEER=2.0 #

def saturate(value, min, max):
    if value<=min:
        return min
    elif value >=max:
        return max
    else:
        return value

class ChaseBall:
    def __init__ (self):
        #angle
        self.blob_x=10 
             
        #depth
        self.blob_y=-20
        #self.blob_depth=10.0
        self._time_detected=0.0

        #self.sub_center=rospy.Subscriber("/blob/point_blob",Float32MultiArray, self.update_ball)
        rospy.loginfo("Subsribers set")

        self.pub_twist=rospy.Publisher("cmd_vel",Twist,queue_size=5)
        rospy.loginfo("Publisher set")

        self._message=Twist()

        self._time_steer=0
        self._steer_sign_prev=0

    def is_detected(self):
        return time.time()-self._time_detected<1.0

    def update_ball(self,message):
        self.blob_x=message[0]
        self.blob_y=message[1]
        self.blob_depth=message[1]
        self._time_detected=time.time()

    def get_control_action(self):
        steer_action=0.0
        throttle_action=0.0

        if self.is_detected:
            #throttle_action =x
            #steer_action = anglular
            steer_action=-K_LAT_DIST_TO_STEER*self.blob_x
            steer_action=saturate(steer_action,-1.5,1.5)
            rospy.loginfo("Steering command %.2f"%steer_action)
            throttle_action=self.blob_y
        return (steer_action, throttle_action)
    
    def run(self):
        rate=rospy.Rate(5)

        while not rospy.is_shutdown():

            #Get the control action
            steer_action, throttle_action=self.get_control_action()
            rospy.loginfo("Steering=%3.1f"%(steer_action))

            self._message.linear.x=throttle_action
            self._message.angular.z=steer_action

            #publish it
            self.pub_twist.publish(self._message)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("chase_ball")
    chase_ball=ChaseBall()
    chase_ball.run()
