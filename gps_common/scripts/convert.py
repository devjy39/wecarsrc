#!/usr/bin/env python
# Translates from NavSatFix to GPSFix and back

import rospy
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import GPSMessage

navsat_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)

def navsat_callback(gps_msg):
    navsat_msg = NavSatFix()
    navsat_msg.header.frame_id = 'map'
    navsat_msg.latitude = gps_msg.latitude
    navsat_msg.altitude = gps_msg.altitude
    navsat_msg.longitude = gps_msg.longitude
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.status.status = 1
    navsat_pub.publish(navsat_msg)

if __name__ == '__main__':
    rospy.init_node('navsat_convertor', anonymous=True)
    navsat_sub = rospy.Subscriber("/gps",GPSMessage, navsat_callback)
    rospy.spin()

