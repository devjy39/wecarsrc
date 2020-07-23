#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform,STOPLineEstimator

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("image_jpeg/compressed",CompressedImage, self.callback)
        self.img_wlane = None
    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([25,0,245])
        upper_wlane = np.array([35,30,255])
        self.img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        self.img_wlane[int(0.7*img_hsv.shape[0]):, :] = 0

if __name__ == "__main__":
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('stopline_detector', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam= params_cam)
    sline_detector = STOPLineEstimator()
    
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.img_wlane is not None:

            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            sline_detector.get_x_points(lane_pts)
            sline_detector.estimate_dist(0.3)

            # sline_detector.visualize_dist()

            sline_detector.pub_sline()

            rate.sleep()