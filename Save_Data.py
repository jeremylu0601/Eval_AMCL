#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
import csv

def callback1(data):
    t=rospy.get_time()
    with open('amcl.csv','a',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([t,data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    rospy.loginfo('AMCL')

def callback(data):
    t=rospy.get_time()
    with open('gt.csv','a',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([t,data.pose[2].position.x,data.pose[2].position.y,data.pose[2].orientation.z,data.pose[2].orientation.w])
    rospy.loginfo('gt')

def listener():
    rospy.init_node('DATA_SAVE', anonymous=True)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback1)
    rospy.spin()

if __name__ == '__main__':
    listener()
