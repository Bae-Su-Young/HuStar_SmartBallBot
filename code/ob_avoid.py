#!/usr/bin/env python2
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

from numpy.core.fromnumeric import argmax, argmin
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from math import radians, degrees, sqrt, pi

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler # not used

class Avoid_obstacle():
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.yaw = 0 # direction

        self.scan_sample = 72 # 360 / 72 = 5 degrees
        self.one_sample_rad = 2 * math.pi / self.scan_sample

        self.boundary_idx = [4, 32, 39, 67] # near vertex
        self.machine_zone = [] # machine volume

        self.half_width = 0.15 # width 0.3 / 2
        self.forward_len = 0.4
        self.backward_len = 0.5

        self.dangerous_zone = 0.15 # temporary value
        self.warning_zone = 0.3 # temporary value
        
        self.linear_vel = 0.2 # temporary value, linear velocity
        self.angular_vel = 0.2 # temporary value, angular velocity

        self.calc_zone() # machine_zone calculate

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.angle = euler_from_quaternion(orientation_list)
        # self.angle = round(self.angle, 2)

    def getNormalizedRadian(self, angle):
        return_angle = angle        
        if return_angle > pi:
            return_angle -= 2 * pi
        elif return_angle < -pi:
            return_angle += 2 * pi

        return return_angle

    def calc_zone(self):
        for i in range(self.scan_sample):
            if i <= self.boundary_idx[0]:
                self.machine_zone.append(self.forward_len / math.cos(self.one_sample_rad * i))
            elif i <= self.boundary_idx[1]:
                self.machine_zone.append(self.half_width / math.sin(self.one_sample_rad * i))
            elif i <= self.boundary_idx[2]:
                self.machine_zone.append(- self.backward_len / math.cos(self.one_sample_rad * i))
            elif i <= self.boundary_idx[3]:
                self.machine_zone.append(- self.half_width / math.sin(self.one_sample_rad * i))
            else:
                self.machine_zone.append(self.forward_len / math.cos(self.one_sample_rad * i))
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_temp = scan.ranges # 360
        scan_filter = []	
	
        for i in range(len(scan_temp)): # 360 to 72
            if i % 5 == 0:
                scan_filter.append(scan_temp[i])

        for i in range(self.scan_sample):
            if scan_filter[i] == float('Inf') or scan_filter[i] == 0: # Inf value preprocessing
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]): # nan value preprocessing
                scan_filter[i] = 0
        
        return scan_filter

    # second method ; only observe forward lidar distance value, and return head direction
    def avoid_obstacle_2(self):
        twist = Twist()

        turn_state_flag = 0 # 0 : forward, 1 : left, 2 : right
        turn_start_time_flag = False
        start_turn_time = 0
        finish_turn_time = 0
        turn_process_done = False

        straight_process_done = False

        return_start_time_flag = False
        start_return_time = 0
        return_process_done = False

        stop_flag = False
        start_angle = 0
        end_angle = 0
        temp_end_angle = 0

        while not rospy.is_shutdown() and not return_process_done:
            lidar_distances = self.get_scan()
            forward_real_distances = [lidar_distances[i] - self.machine_zone[i] for i in range(-5, 5)]
            warning_idx = []
            for i in range(len(forward_real_distances)):
                if forward_real_distances[i] < self.warning_zone:
                    warning_idx.append(i)

            if not turn_start_time_flag:
                start_turn_time = time.time()
                turn_start_time_flag = True
            elif not turn_process_done:
                finish_turn_time = time.time()            

            if not return_start_time_flag and straight_process_done:
                start_return_time = time.time()
                return_start_time_flag = True

            if not turn_process_done: # stop and turning
                if turn_state_flag == 1:
                    twist.linear.x = 0
                    twist.angular.z = - self.angular_vel
                elif turn_state_flag == 2:
                    twist.linear.x = 0
                    twist.angular.z = self.angular_vel
            
                if turn_state_flag == 0:
                    if len(warning_idx) == 0:
                        turn_state_flag = 0
                    elif warning_idx[0] > (len(forward_real_distances) - warning_idx[len(warning_idx) - 1] - 1): # probably?
                        turn_state_flag = 1
                    else:
                        turn_state_flag = 2

                if len(warning_idx) < 1:
                    turn_process_done = True
                    twist.linear.x = 0
                    twist.angular.z = 0
                print('here 1')
                
            elif not straight_process_done: # side through obstacle 
                print('here 2')
                real_distances = [lidar_distances[i] - self.machine_zone[i] for i in range(len(lidar_distances))] # lidar distance - machine volume, distance between machine to obstacle
                min_distance = min(real_distances) # most near distance
                min_idx = argmin(real_distances)

                if min_distance <= self.dangerous_zone:
                    stop_flag = True
                    straight_process_done = True

                    start_angle = self.angle
                    if turn_state_flag == 1:
                        end_angle = start_angle - pi / 6
                        temp_end_angle = end_angle - 0.1
                    elif turn_state_flag == 2:
                        end_angle = start_angle + pi / 6
                        temp_end_angle = end_angle + 0.1

                    end_angle = self.getNormalizedRadian(end_angle)
                    temp_end_angle = self.getNormalizedRadian(temp_end_angle)

                    if (turn_state_flag == 1 and end_angle > temp_end_angle) or (turn_state_flag == 2 and end_angle < temp_end_angle):
                        end_angle, temp_end_angle = temp_end_angle, end_angle
                    
                elif not min_idx == 18 and not min_idx == 54:
                    twist.linear.x = self.linear_vel
                    twist.angular.z = 0
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                    straight_process_done = True
                    # print(return_start_time_flag)

            elif not return_process_done and not stop_flag: # return direction
                if (finish_turn_time - start_turn_time) > (time.time() - start_return_time):
                    if turn_state_flag == 1:
                        twist.linear.x = 0
                        twist.angular.z = self.angular_vel
                        print('1')
                    elif turn_state_flag == 2:
                        twist.linear.x = 0
                        twist.angular.z = - self.angular_vel
                        print('2')
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                    return_process_done = True
                    print('3')

            elif stop_flag:
                if turn_state_flag == 1:
                    if not (temp_end_angle > self.angle and end_angle < self.angle):
                        twist.linear.x = 0
                        twist.angular.z = - self.angular_vel
                elif turn_state_flag == 2:
                    if not (temp_end_angle < self.angle and end_angle > self.angle):
                        twist.linear.x = 0
                        twist.angular.z = self.angular_vel
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                    return_process_done = True

            self.pub_cmd_vel.publish(twist)

    def get_avoid_obstacle_flag(self):
        lidar_distances = self.get_scan()
        forward_real_distances = [lidar_distances[i] - self.machine_zone[i] for i in range(-5, 5)]
        warning_idx = []
        
        return_value = False

        for i in range(len(forward_real_distances)):
            if forward_real_distances[i] < self.warning_zone:
                warning_idx.append(i)

        if len(warning_idx) > 0:
            return_value = True
        
        print(return_value)

        return return_value