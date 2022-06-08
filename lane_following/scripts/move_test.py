#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 上面两行不可省略，第一行是：告诉操作系统执行这个脚本的时候，调用 /usr/bin 下的 python 解释器。第二行是：定义编码格式 "UTF-8-" 支持中文

from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


def send_goals_python():
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    twist = Twist()
    time.sleep(1)
    twist.linear.x = 0.2
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    time.sleep(4)
    twist.linear.x = 0
    twist.angular.z = -2
    cmd_vel_pub.publish(twist)
    time.sleep(0.8)
    twist.linear.x = 0.2
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    time.sleep(1)
    twist.linear.x = 0.2
    twist.angular.z = 1
    cmd_vel_pub.publish(twist)
    time.sleep(2.5)
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

    return "Mission Finished."


if __name__ == "__main__":
    rospy.init_node("send_goals_python")  # python 语言方式下的　初始化 ROS 节点，
    result = send_goals_python()
    rospy.loginfo(result)
