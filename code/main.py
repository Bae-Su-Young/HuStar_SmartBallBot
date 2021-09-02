#!/usr/bin/env python2
import rospy
from ob_avoid import Avoid_obstacle
from chase_the_ball import ChaseBall
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('ball_bot', anonymous=True)
    rate = rospy.Rate(5)
    
    obstacle = Avoid_obstacle()
    chase_ball = ChaseBall()

    while not rospy.is_shutdown():
        if obstacle.get_avoid_obstacle_flag() == True:
            obstacle.avoid_obstacle_2()
            chase_ball.findBall_forward_flag = True
            chase_ball.no_detected_ball = False
        else:
            chase_ball.run()
        rate.sleep()

if __name__ == '__main__':
    main()