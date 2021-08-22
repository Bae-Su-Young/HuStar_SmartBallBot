#!/usr/bin/env python
 # -*- coding: utf-8 -*-
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import os
import json
import numpy as np
import random
import time
import sys
import math
#from math import pi,pow,atan2f
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry


tx=0
reload(sys)
sys.setdefaultencoding('utf8')
def callback(msg):
    global tx
    print(msg)
    tx=msg.position.x
    #position=Pose()
    #position = odom.pose.pose.position



def getChasherAngle(bx,depth,frame_length,tx):
    center_of_frame=frame_length//2
    move_ang=0

    if bx-tx >-10 and bx-tx<-10:
        move_ang=0
        return move_ang
    else:
        tan1=math.fabs(bx-tx)
        tan2=math.sqrt(math.fabs(math.pow(depth,2)-math.pow(tan1,2)))
        tan2=tan2.real*180/3.1415
        move_ang=math.atan2(float(tan1),float(tan2))
        return move_ang

if __name__ == '__main__':
    rospy.init_node('realdrive_dqn_baesu')
    sub_geo = rospy.Subscriber('/turtle/gt_pose',Pose, callback)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)


    start_time=time.time()

    while True:
        done=False
        while not done:
            # while True:
            ball_x=50.0
            ball_depth=3.0
            
            #result_angle=getChasherAngle(ball_x,ball_depth,1000,tx) /100
            result_angle=0
            #print(result_angle)
            vel_cmd = Twist() 
            vel_cmd.linear.x = ball_depth 
            vel_cmd.angular.z = result_angle
            print('ball_depth: ',ball_depth,'angular: ',result_angle)
            pub_cmd_vel.publish(vel_cmd)
                
        