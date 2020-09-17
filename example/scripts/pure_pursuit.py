#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,pow,sqrt,atan2
from geometry_msgs.msg import Point32,PoseStamped,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class pure_pursuit:

    def __init__(self):
        rospy.init_node('pure_pursuit',anonymous=True)
        rospy.Subscriber('/local',Path,self.path_callback)
        rospy.Subscriber('/sensors/core', VescStateStamped, self.speed_callback)

        rospy.Subscriber('/odom',Odometry, self.odom_callback)
        #rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.motor_msg = Float64()
        self.servo_msg= Float64()
        self.is_path = False
        self.is_odom = False
        self.is_amcl = False
        self.forward_point = Point()
        self.current_position = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 1
        self.lfd = 0.0
        self.min_lfd = 1.0
        self.max_lfd = 15.0
        self.steering = Float64()
        self.speed = Float64()

        self.steering_angle_to_servo_gain = -1.2135
        self.steering_angle_to_servo_offset = 0.5304
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():

            if self.is_path == True and (self.is_odom==True or self.is_amcl ==True):

                vehicle_position = self.current_position
                rotated_point = Point()

                self.is_look_forward_point = False
                
                for num,i in enumerate(self.path.poses):
                    path_point = i.pose.position
                    dx = path_point.x - vehicle_position.x
                    dy = path_point.y - vehicle_position.y
                    rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
                    rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy

                    if rotated_point.x>0:
                        dis = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                        if dis >= self.lfd:
                            self.forward_point = path_point
                            self.is_look_forward_point = True
                            break
                theta = -atan2(rotated_point.y,rotated_point.x)

                if self.is_look_forward_point:
                    s = (self.speed / 4616) * 3.6
                    if(s<10):
                        self.lfd = self.min_lfd
                    elif(s>10 and s<70):
                        self.lfd = s/6
                    else:
                        self.lfd = self.max_lfd

                    self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)
                    print(self.steering*180/pi, self.lfd)
                    #self.motor_msg.data = 76933
                else:
                    self.steering=0
                    print("no found forward point")
                    self.motor_msg.data = 0

                self.steering_command = (self.steering_angle_to_servo_gain*self.steering)+self.steering_angle_to_servo_offset
                self.servo_msg.data = self.steering_command

                self.servo_pub.publish(self.servo_msg)
                #self.motor_pub.publish(self.motor_msg)
            rate.sleep()

    def speed_callback(self,msg):
        self.speed = msg.state.speed

    def path_callback(self,msg):
        self.is_path = True
        self.path = msg
    
    def odom_callback(self,msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def amcl_callback(self,msg):
        self.is_amcl=True
        amcl_quaternion= (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(amcl_quaternion)
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y

if __name__ == "__main__":
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass




