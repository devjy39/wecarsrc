#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import pandas as pd

class path_pub:

    def __init__(self):
        rospy.init_node('make_pub',anonymous=True)

        self.path_pub = rospy.Publisher('/ground', PointCloud, queue_size=1)
        self.path_msg = PointCloud()
        self.path_msg.header.frame_id = '/map'
        
        self.offset_x=302459.942
        self.offset_y=4122635.537

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('gps_common')
        full_path = pkg_path+'/kcity_PM0138'+'/A1LANE_CenterLine_0001.csv'

        #self.f = pd.read_csv(full_path) 
        self.read_centerline(pkg_path)
        self.read_normallane(pkg_path)
        self.read_roadedge(pkg_path)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.path_pub.publish(self.path_msg)
            rate.sleep()

    def read_centerline(self,pkg_path):
        for i in range(1,352):
            if(i<10):
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_CenterLine_000'+str(i)+'.csv'
            elif(i<100):
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_CenterLine_00'+str(i)+'.csv'
            else:
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_CenterLine_0'+str(i)+'.csv'

            f= open(path,'r')
            lines = f.readlines()

            i = 0
            for line in lines:
                if(i>7):
                    tmp = line.split()
                    read_pose=Point32()

                    read_pose.x=float(tmp[0])-self.offset_x
                    read_pose.y=float(tmp[1])-self.offset_y
                    read_pose.z=0
                    #print(read_pose)
                    self.path_msg.points.append(read_pose)
                i += 1

            f.close()

    def read_normallane(self,pkg_path):
        for i in range(1,187):
            if(i<10):
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_NormalLane_000'+str(i)+'.csv'
            elif(i<100):
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_NormalLane_00'+str(i)+'.csv'
            else:
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_NormalLane_0'+str(i)+'.csv'

            f= open(path,'r')
            lines = f.readlines()

            i = 0
            for line in lines:
                if(i>7):
                    tmp = line.split()
                    read_pose=Point32()
                    read_pose.x=float(tmp[0])-self.offset_x
                    read_pose.y=float(tmp[1])-self.offset_y
                    read_pose.z=0
                    #print(read_pose)
                    self.path_msg.points.append(read_pose)
                i += 1

            f.close()

    def read_roadedge(self,pkg_path):
        for i in range(1,561):
            if(i<10):
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_RoadEdge_000'+str(i)+'.csv'
            elif(i<100):
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_RoadEdge_00'+str(i)+'.csv'
            else:
                path = pkg_path+'/kcity_PM0138'+'/A1LANE_RoadEdge_0'+str(i)+'.csv'

            f= open(path,'r')
            lines = f.readlines()

            i = 0
            for line in lines:
                if(i>7):
                    tmp = line.split()
                    read_pose=Point32()
                    read_pose.x=float(tmp[0])-self.offset_x
                    read_pose.y=float(tmp[1])-self.offset_y
                    read_pose.z=0
                    #print(read_pose)
                    self.path_msg.points.append(read_pose)
                i += 1

            f.close()
            
            


if __name__ == "__main__":
    try:
        test_track=path_pub()
    except rospy.ROSInterruptException:
        pass

