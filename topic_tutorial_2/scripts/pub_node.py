#!/usr/bin/env python
import rospy
from topic_tutorial_2.msg import MyMsgs

NAME_TOPIC = '/msgs_talk'
NAME_NODE = 'pub_node'

if __name__=='__main__':

    rospy.init_node(NAME_NODE, anonymous=True)

    pub = rospy.Publisher(NAME_TOPIC,MyMsgs, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    msgs_pub = MyMsgs()

    while not rospy.is_shutdown():
            
        for i in range(4):
            msgs_pub.x[i] = 10*(i+1)
            msgs_pub.y[i] = 10*(i+1)

        pub.publish(msgs_pub)

        rate.sleep()