#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 上面两行不可省略，第一行是：告诉操作系统执行这个脚本的时候，调用 /usr/bin 下的 python 解释器。第二行是：定义编码格式 "UTF-8-" 支持中文

from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose


def send_goals_python():
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    twist = Twist()

    # 定义四个发送目标点的对象
    goal1 = MoveBaseGoal()
    goal3 = MoveBaseGoal()
    goal4 = MoveBaseGoal()
    goal5 = MoveBaseGoal()
    goal6 = MoveBaseGoal()
    goal7 = MoveBaseGoal()
    goal8 = MoveBaseGoal()
    goal9 = MoveBaseGoal()
    # 初始化四个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》

    goal1.target_pose.pose.position.x = -0.677
    goal1.target_pose.pose.position.y = -4.183
    goal1.target_pose.pose.orientation.z = 0.005
    goal1.target_pose.pose.orientation.w = 1

    goal3.target_pose.pose.position.x = 0.543
    goal3.target_pose.pose.position.y = -3.465
    goal3.target_pose.pose.orientation.z = 0.016
    goal3.target_pose.pose.orientation.w = 1

    goal4.target_pose.pose.position.x = 3.254
    goal4.target_pose.pose.position.y = -4.327
    goal4.target_pose.pose.orientation.z = -0.015
    goal4.target_pose.pose.orientation.w = 1

    goal5.target_pose.pose.position.x = 3.532
    goal5.target_pose.pose.position.y = -4.502
    goal5.target_pose.pose.orientation.z = 0.705
    goal5.target_pose.pose.orientation.w = 0.710

    goal6.target_pose.pose.position.x = 3.551
    goal6.target_pose.pose.position.y = -4.234
    goal6.target_pose.pose.orientation.z = 0.674
    goal6.target_pose.pose.orientation.w = 0.738

    goal7.target_pose.pose.position.x = 3.593
    goal7.target_pose.pose.position.y = -2.720
    goal7.target_pose.pose.orientation.z = 0.707
    goal7.target_pose.pose.orientation.w = 0.707

    goal8.target_pose.pose.position.x = 1.929
    goal8.target_pose.pose.position.y = -2.313
    goal8.target_pose.pose.orientation.z = 1
    goal8.target_pose.pose.orientation.w = 0.015

    goal9.target_pose.pose.position.x = 1.425
    goal9.target_pose.pose.position.y = -3.556
    goal9.target_pose.pose.orientation.z = -0.705
    goal9.target_pose.pose.orientation.w = 0.710

    goal_p1 = [
        goal3,
    ]
    goal_p2 = [
        goal4,
        goal5,
        goal6,
        goal7,
    ]
    goal_p3 = [
        goal8,
    ]
    goal_p4 = [
        goal9,
    ]
    goal_p5 = [
        goal1,
    ]
    time.sleep(0.5)
    twist.linear.x = 0.2
    twist.angular.z = 0.4
    cmd_vel_pub.publish(twist)
    time.sleep(4)
    # twist.linear.x = 0
    # twist.angular.z = 0
    # cmd_vel_pub.publish(twist)

    goal_p1[0].target_pose.header.frame_id = "map"
    goal_p1[0].target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal_p1[0])
    str_log = "Send NO. %s Part !!!" % str(1)
    rospy.loginfo(str_log)
    wait = client.wait_for_result()
    if wait:
        str_log = "The NO. %s Part achieved success !!!" % str(1)
        rospy.loginfo(str_log)
        time.sleep(1)
    # twist.linear.x = 0
    # twist.angular.z = -0.5
    # cmd_vel_pub.publish(twist)
    # time.sleep(0.5)
    nowQua = Pose().orientation.z
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
    time.sleep(1.3)
    twist.linear.x = 0.2
    twist.angular.z = 1
    cmd_vel_pub.publish(twist)
    time.sleep(2.5)
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

    str_log = "Send NO. %s Part !!!" % str(2)
    rospy.loginfo(str_log)
    for i in range(0, 4):
        goal_p2[i].target_pose.header.frame_id = "map"
        goal_p2[i].target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal_p2[i])

        str_log = "Send NO. %s Goal !!!" % str(i)
        rospy.loginfo(str_log)

        wait = client.wait_for_result()
        if wait:
            str_log = "The NO. %s Goal achieved success !!!" % str(i)
            rospy.loginfo(str_log)
            time.sleep(1)
    str_log = "The NO. %s Part achieved success !!!" % str(2)
    rospy.loginfo(str_log)

    # goal_p3[0].target_pose.header.frame_id = "map"
    # goal_p3[0].target_pose.header.stamp = rospy.Time.now()
    # client.send_goal(goal_p3[0])
    # str_log = "Send NO. %s Part !!!" % str(3)
    # rospy.loginfo(str_log)
    # wait = client.wait_for_result()
    # if wait:
    #     str_log = "The NO. %s Part achieved success !!!" % str(3)
    #     rospy.loginfo(str_log)
    #     time.sleep(1)

    # goal_p4[0].target_pose.header.frame_id = "map"
    # goal_p4[0].target_pose.header.stamp = rospy.Time.now()
    # client.send_goal(goal_p4[0])
    # str_log = "Send NO. %s Part !!!" % str(4)
    # rospy.loginfo(str_log)
    # wait = client.wait_for_result()
    # if wait:
    #     str_log = "The NO. %s Part achieved success !!!" % str(4)
    #     rospy.loginfo(str_log)
    #     time.sleep(1)

    # goal_p5[0].target_pose.header.frame_id = "map"
    # goal_p5[0].target_pose.header.stamp = rospy.Time.now()
    # client.send_goal(goal_p5[0])
    # str_log = "Send NO. %s Part !!!" % str(5)
    # rospy.loginfo(str_log)
    # wait = client.wait_for_result()
    # if wait:
    #     str_log = "The NO. %s Part achieved success !!!" % str(5)
    #     rospy.loginfo(str_log)
    #     time.sleep(1)
    return "Mission Finished."


if __name__ == "__main__":
    rospy.init_node("send_goals_python", anonymous=True)  # python 语言方式下的　初始化 ROS 节点，
    # while True:
    result = send_goals_python()
    rospy.loginfo(result)
