#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys,os
import time,datetime
import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


def odom2globalPlan():
    #rospy.init_node("pubpath", anonymous=True)         			# 初始化节点 名称:pubpath
    path_pub = rospy.Publisher('/test_path',Path, queue_size=10)    # 发布节点名称: followingpath
    path = Path()                                                       # 初始化消息类型,Path
    rospy.init_node("pubpath", anonymous=True)         			# 初始化节点 名称:pubpath
    rate = rospy.Rate(0.5)							# 设置ros发布频率，单位hz。频率不能太快，要低于1HZ。否则purepsuit节点会导致tf转换不过来进而报错
    path.header.frame_id = "map"
    path.header.stamp = rospy.get_rostime()					# 获取ros当前时间
    pathList = []
    with open(pathfile,'r') as lines:
        for line in lines:
            rows = line.strip('\n').split('|')
            posestamped = PoseStamped()               # 初始化消息类型,PoseStamped
            #posestamped.header.stamp = rospy.get_rostime()
            #posestamped.header.frame_id = "map"
            posestamped.pose.position.x = float(rows[0])
            posestamped.pose.position.y = float(rows[1])
            posestamped.pose.position.z = 0.0
            posestamped.pose.orientation.w = 1.0
            path.poses.append(posestamped)
    
    while not rospy.is_shutdown():
            path_pub.publish(path)
            rate.sleep()




if __name__ == '__main__':
    pathfile = "/home/driver/Ackermann-chassis-PID/src/following/path_dir/wps.txt"
    try:
        odom2globalPlan()
    except rospy.ROSInterruptException:
        pass
