#!/usr/bin/env python

import roslib
import sys
import rospy
from std_msgs.msg import Float64
import time

class motor_control:
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.timer_to_sending_data = 0
        
        self.speed = rospy.Publisher('/commands/motor/speed', Float64,  queue_size=1)
        self.position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

        p = 0.1
        l = 1
        while not rospy.is_shutdown():
            
            self.speed_value = 1700
            self.position_value = p
            self.speed.publish(self.speed_value)
            self.position.publish(self.position_value)
            self.rate.sleep()
            print(p,l)
            if(l==1):
                if(p>0.9):
                    l = 0
                else:
                    p += 0.01
            else:
                if (p<0.1):
                    l = 1
                else:
                    p -= 0.01

def main(args):

    rospy.init_node('motor_control', anonymous = True)

    motor_control()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)