#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import math
import numpy as np
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,pow,sqrt
from geometry_msgs.msg import Point32,PoseStamped,Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import ObjectInfo
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class cruiseControl:
    def __init__(self, object_vel_gain, object_dis_gain):
        self.object = [False,0]
        self.stop_lane= [False,0]
        self.object_vel_gain = object_vel_gain
        self.object_dis_gain = object_dis_gain

    def checkObject(self,ref_path, global_vaild_object, local_vaild_object):
        self.object=[False,0]
        self.stop_lane=[False,0]

        if len(global_vaild_object) > 0:
            min_rel_distance = float('inf')

            for i in range(len(global_vaild_object)):

                for path in ref_path.poses:
                    if global_vaild_object[i][0] == 2:
                        dis =sqrt(pow(path.pose.position.x - global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                        if dis<2:
                            rel_distance = sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.stop_lane=[False,0]
                                self.object=[True,i]
                            break
                    elif global_vaild_object[i][0] == 1:
                        dis =sqrt(pow(path.pose.position.x - global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                        if dis<1.5:
                            rel_distance = sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.stop_lane=[True,0]
                                self.object=[False,i]

    def acc(self, local_vaild_object, ego_vel, target_vel):
        out_vel = target_vel

        if self.object[0] == True:
            print("ACC ON")
            front_vehicle = [local_vaild_object[self.object[1]][1],local_vaild_object[self.object[1]][2],local_vaild_object[self.object[1]][3]]
            time_gap = 1
            default_space = 2
            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-4.4
            print(dis_safe,dis_rel)

            vel_rel = (front_vehicle[2] - ego_vel) * 3.6
            v_gain = self.object_vel_gain
            x_errgain = self.object_dis_gain
            accelation = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            acc_based_vel = ego_vel * 3.6 + accelation

            if acc_based_vel > target_vel:
                acc_based_vel = target_vel

            if acc_based_vel < 1:
                acc_based_vel = 0
            
            if dis_safe - dis_rel > 0:
                out_vel = acc_based_vel
            else:
                if acc_based_vel < target_vel:
                    out_vel = acc_based_vel
        return out_vel




class vaildObject:

    def __init__(self):
        self.vaild_stoplane_position=[[58.26, 1180.09], [85.56, 1228.28]]

    def get_object(self,obj_msg):
        self.all_object = ObjectInfo()
        self.all_object = obj_msg

    def calc_vaild_obj(self,ego_pose):
        global_object_info = []
        local_object_info = []

        if self.all_object.num_of_objects > 0:
            tmp_theta = ego_pose[2]
            tmp_translation = [ego_pose[0],ego_pose[1]]
            tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]],
                [0,0,1]])
            tmp_det_t = np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])],
                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])],
                [0,0,1]])
        
            for num in range(self.all_object.num_of_objects):
                global_result = np.array([[self.all_object.pose_x[num]], [self.all_object.pose_y[num]], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0]>0:
                    global_object_info.append([self.all_object.object_type[num], self.all_object.pose_x[num],self.all_object.pose_y[num], self.all_object.velocity[num]])
                    local_object_info.append([self.all_object.object_type[num],local_result[0][0],local_result[1][0], self.all_object.velocity[num]])

            for num in range(len(self.vaild_stoplane_position)):
                global_result = np.array([[self.vaild_stoplane_position[num][0]],[self.vaild_stoplane_position[num][1]], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0]>0:
                    global_object_info.append([1,global_result[0],global_result[1], 0])
                    local_object_info.append([1,local_result[0][0], local_result[1][0], 0])
        
        return global_object_info,local_object_info   


class velocityPlanning:

    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curveBaseVelocity(self,global_path,point_num):
        out_vel_plan = []
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)
        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append([-(x*x)-(y*y)])

            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)*3.6
            if v_max>self.car_max_speed:
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)

        return out_vel_plan

class path_pub:

    def __init__(self):
        rospy.init_node('make_pub',anonymous=True)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        self.path_msg = Path()
        self.path_msg.header.frame_id = '/map'

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('gps_common')
        self.full_path = pkg_path+'/path'+'/path.txt'
        self.f= open(self.full_path,'r')
        lines = self.f.readlines()

        rospy.Subscriber('/odom',Odometry, self.odom_callback)
        self.is_odom = False
        self.vehicle_yaw = Float64()
        self.current_position = Point()
        self.local_pub = rospy.Publisher('/local', Path, queue_size=1)
        self.local_msg = Path()
        self.local_msg.header.frame_id = '/map'

        self.vel_pub = rospy.Publisher('target_vel',Float64, queue_size=1)

        self.v = vaildObject()
        rospy.Subscriber('/obj_info', ObjectInfo, self.obj_callback)
        self.ego = []
        cruise_control = cruiseControl(1,0.5)

        rospy.Subscriber('/sensors/core',VescStateStamped,self.status_callback)
        self.speed = Float64()

        for line in lines:
            tmp = line.split()
            read_pose=PoseStamped()
            x = float(tmp[0])
            y = float(tmp[1])
            read_pose.pose.position.x= x
            read_pose.pose.position.y= y
            read_pose.pose.orientation.w = 1
            self.path_msg.poses.append(read_pose)
        self.f.close()

        v=velocityPlanning(80,0.2)
        vel = v.curveBaseVelocity(self.path_msg,100)    

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_odom == True :
                self.path_pub.publish(self.path_msg)
                i = self.find_local()+10   #10개인덱스뒤에꺼로 속도보내게해놈
                if(i >= len(vel)):
                    i = len(vel)

                target_vel = vel[i]

                self.ego = [self.current_position.x, self.current_position.y, self.vehicle_yaw]
                global_vaild_obj,local_vaild_obj= self.v.calc_vaild_obj(self.ego)
                cruise_control.checkObject(self.local_msg,global_vaild_obj,local_vaild_obj)
                target_vel = cruise_control.acc(local_vaild_obj,self.speed,target_vel)
                self.vel_pub.publish(target_vel)
                print("speed = ",target_vel)
        
            rate.sleep()

    def find_local(self):
        self.local_msg = Path()
        self.local_msg.header.frame_id = '/map'
        f= open(self.full_path,'r')
        lines = f.readlines()
        distance = Float64()
        car = 0

        for index,line in enumerate(lines):
            tmp = line.split()
            x = float(tmp[0])
            y = float(tmp[1])
            dis_tmp = math.sqrt(math.pow((x-self.current_position.x),2) +
                math.pow((y-self.current_position.y),2))
            if(dis_tmp < distance):
                distance = dis_tmp
                car = index

        for i in range(car,car+40):
            if(i>=len(lines)):
                break
            tmp = lines[i].split()
            read_pose=PoseStamped()
            x = float(tmp[0])
            y = float(tmp[1])
            read_pose.pose.position.x= x
            read_pose.pose.position.y= y
            read_pose.pose.orientation.w = 1
            self.local_msg.poses.append(read_pose)

        self.local_pub.publish(self.local_msg)
        f.close()
        return car

    def status_callback(self,msg):
        rpm = msg.state.speed
        self.speed = rpm/4614

    def obj_callback(self,msg):
        self.v.get_object(msg)

    def odom_callback(self,msg):
        self.local_msg = Path()
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        

if __name__ == "__main__":
    try:
        test_track=path_pub()

    except rospy.ROSInterruptException:
        pass

