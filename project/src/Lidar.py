import rospy
import numpy as np
import math
from math import pi

from sensor_msgs.msg import LaserScan
from std_msgs.msg import bool

MEASUREMENTS=4

class Lidar_Filter:
    def __init__(self):
        self.laser_filtered_pub=rospy.Publisher('/scan_filter',LaserScan,queue_size=5)
        self.scan_sub=rospy.Subscriber('/scan',LaserScan,self.process_laser)
        self.scan_values=[]
        inf=float("inf")
        self.scan_values_2=[[inf for col in range(360)] for row in range(MEASUREMENTS)]
        self.counter = 0        # is used to count the number of measurements.
        self.init = False       # is used for the second method. We wait if we do not have any measurements at the beginning.
        self.init_counter = 0   # initilization counter to at first receive some measurements results.
        rospy.spin()

    def process_laser(self,data):
        # if self.count<MEASUREMENTS:
        #     self.scan_values.append(data.ranges)
        #     self.counter+=1
        # else:
        #     self.filter_values()
        counter=data.header.seq % MEASUREMENTS
        self.scan_values_2[counter]=data.ranges
        self.filter_values_2(counter)

    def filter_values_2(self,counter):
        inf=float("inf")scan
            if last_mea[i]==inf:
                for k in range(1,MEASUREMENTS):
                    if self.scan_values_2[counter-k]!=inf:
                        last_mea[i]=self.scan_values_2[k][i]
                        break
        scan=LaserScan()
        scan.ranges=last_mea
        self.laser_filtered_pub.publish(scan)




def run():
    rospy.init_node('lidar_filter')
    filter=Lidar_Filter()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

        



 