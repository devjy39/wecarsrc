#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class wallstop :
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.laser_callback)

        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud, queue_size=1)

        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self,msg):
        pcd = PointCloud()
        motor_msg=Float64()
        servo_msg = Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle= 0

        for r in msg.ranges:
            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            #print(angle,tmp_point.x,tmp_point.y)
            angle=angle+(1.0/180*pi)
            if r<12:
                pcd.points.append(tmp_point)

        count = 0
        for point in pcd.points:
            #print(point.x,point.y)

            if(point.y > -0.5 and point.y< 0.5):
                if point.x >0 and point.x <2:
                    count = count +1
        

        s = 6000
        l = 0
        r = 0
        go = 0

        if count>20:
            for point in pcd.points:
                if point.x > 0 and point.x <3:
                    if point.y > -1 and point.y < 0:
                        l += 1
                    elif point.y > 0 and point.y < 1:
                        r += 1
                    elif point.y > -1 and point.y < 1:
                        go += 1

            print(l,r,go)

            if(l>r):
                print("left")
                servo_msg.data = 0.15
                motor_msg.data= 4000
            else:
                print("right")
                servo_msg.data = 0.95
                motor_msg.data= 4000
            if go == 0:
                print("go")
                motor_msg.data= 5000
                
        else:
            servo_msg.data = 0.53
            motor_msg.data= s

        print(count)
        self.servo_pub.publish(servo_msg)
        self.motor_pub.publish(motor_msg)
        self.pcd_pub.publish(pcd)
    

if __name__ == "__main__":
    try:
        test_track=wallstop()
    except rospy.ROSInterruptException:
        pass

