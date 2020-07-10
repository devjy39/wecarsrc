#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

#def callback(data):
#    for i in range (350, 357):
#      print("%dth value" %i, data.ranges[i] )
#    print('')
   
# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
global pre_po
def callback(data):
    global pre_po
    position_value = pre_po
    
    direc = {'right':0,'left':0}

    for i in range (0, 360):
        lidar_dis = data.ranges[i]
        if(i>100 and i<180 and data.ranges[i]<1.0):
            direc['left'] += 1
        elif(i>180 and i<230 and data.ranges[i] <1.0 ):
            direc['right'] += 1

        if(i%30 == 0):
            print("%dth value" %i, lidar_dis)
    print('')

    if direc['right']==0 and direc['left'] ==0:
        print("go")
        speed_value = 1500
        speed.publish(speed_value)
        # position.publish(position_value)
    elif direc['right'] >0 and direc['left'] ==0:
        print("left turn")
        speed_value = 1500
        speed.publish(speed_value)
        if(position_value <0.7):
            position_value += 0.1
            position.publish(position_value)
    elif direc['right'] ==0 and direc['left'] >0:
        print("right turn")
        speed_value = 1500
        speed.publish(speed_value)
        if(position_value > 0.1):
            position_value -= 0.1
            position.publish(position_value)
    else:
        print("stop")
        speed_value = 0
        speed.publish(speed_value)
        #position.publish(0.5)
        # rate.sleep()
    pre_po = position_value

if __name__ == '__main__':
    rospy.init_node('Check', anonymous=True)

    rate = rospy.Rate(3)
    speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
    position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
    speed_value = 1200
    pre_po = position_value = 0.5
    position.publish(position_value)
    rospy.Subscriber('scan', LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

