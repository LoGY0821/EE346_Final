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
from Follower import Follower


def send_goals_python():
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    # 定义四个发送目标点的对象
    goal0 = MoveBaseGoal()
    goal1 = MoveBaseGoal()
    goal2 = MoveBaseGoal()
    goal3 = MoveBaseGoal()
    goal4 = MoveBaseGoal()
    goal5 = MoveBaseGoal()
    goal6 = MoveBaseGoal()
    goal7 = MoveBaseGoal()
    goal8 = MoveBaseGoal()
    goal9 = MoveBaseGoal()
    goal10 = MoveBaseGoal()
    # 初始化四个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》
    goal0.target_pose.pose.position.x = 0.177
    goal0.target_pose.pose.position.y = -3.492
    goal0.target_pose.pose.orientation.z = -0.006
    goal0.target_pose.pose.orientation.w = 1

    goal1.target_pose.pose.position.x = 0.801
    goal1.target_pose.pose.position.y = -3.471
    goal1.target_pose.pose.orientation.z = -0.029
    goal1.target_pose.pose.orientation.w = 1

    goal2.target_pose.pose.position.x = 3.587
    goal2.target_pose.pose.position.y = -4.419
    goal2.target_pose.pose.orientation.z = 0.738
    goal2.target_pose.pose.orientation.w = 0.675

    goal3.target_pose.pose.position.x = 1.729
    goal3.target_pose.pose.position.y = -2.236
    goal3.target_pose.pose.orientation.z = 0.702
    goal3.target_pose.pose.orientation.w = 0.712

    goal4.target_pose.pose.position.x = 1.865
    goal4.target_pose.pose.position.y = -0.347
    goal4.target_pose.pose.orientation.z = 0.696
    goal4.target_pose.pose.orientation.w = 0.718

    goal5.target_pose.pose.position.x = 3.917
    goal5.target_pose.pose.position.y = -0.488
    goal5.target_pose.pose.orientation.z = 1.000
    goal5.target_pose.pose.orientation.w = -0.031

    goal6.target_pose.pose.position.x = -0.208
    goal6.target_pose.pose.position.y = -0.163
    goal6.target_pose.pose.orientation.z = 0.999
    goal6.target_pose.pose.orientation.w = 0.050

    goal7.target_pose.pose.position.x = 1.934
    goal7.target_pose.pose.position.y = -0.823
    goal7.target_pose.pose.orientation.z = -0.731
    goal7.target_pose.pose.orientation.w = 0.682

    goal8.target_pose.pose.position.x = 1.595
    goal8.target_pose.pose.position.y = -3.611
    goal8.target_pose.pose.orientation.z = 0.999
    goal8.target_pose.pose.orientation.w = 0.052

    goal9.target_pose.pose.position.x = 0.369
    goal9.target_pose.pose.position.y = -3.555
    goal9.target_pose.pose.orientation.z = 0.956
    goal9.target_pose.pose.orientation.w = -0.295

    goal10.target_pose.pose.position.x = -0.666
    goal10.target_pose.pose.position.y = -4.170
    goal10.target_pose.pose.orientation.z = 1.000
    goal10.target_pose.pose.orientation.w = 0.015

    goal_lists = [
        goal0,
        goal1,
        goal2,
        goal3,
        goal4,
        goal5,
        goal6,
        goal7,
        goal8,
        goal9,
        goal10,
    ]  # 采用 python 中的列表方式，替代实现C/C++ 中的数组概念
    goal_number = 0
    while goal_number < len(goal_lists):
        goal_lists[goal_number].target_pose.header.frame_id = "map"
        goal_lists[goal_number].target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal_lists[goal_number])
        str_log = "Send NO. %s Goal !!!" % str(goal_number)
        rospy.loginfo(str_log)

        wait = client.wait_for_result()
        if wait:
            str_log = "The NO. %s Goal achieved success !!!" % str(goal_number)
            rospy.loginfo(str_log)
            goal_number = goal_number + 1
    return "Mission Finished."


if __name__ == "__main__":
    rospy.init_node("send_goals_python", anonymous=True)  # python 语言方式下的　初始化 ROS 节点，
    result = send_goals_python()
    rospy.loginfo(result)
