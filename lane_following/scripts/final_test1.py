#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy, cv2, cv_bridge, numpy, math, time
from actionlib.action_client import GoalManager
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber("camera/image", Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.twist = Twist()
        # self.client.wait_for_server()

        # self.start = 0

        # 定义四个发送目标点的对象
        self.goal1 = MoveBaseGoal()
        self.goal3 = MoveBaseGoal()
        self.goal4 = MoveBaseGoal()
        self.goal5 = MoveBaseGoal()
        self.goal6 = MoveBaseGoal()
        self.goal7 = MoveBaseGoal()
        self.goal8 = MoveBaseGoal()
        self.goal9 = MoveBaseGoal()
        # 初始化四个目标点在 map 坐标系下的坐标

        self.goal1.target_pose.pose.position.x = -0.677
        self.goal1.target_pose.pose.position.y = -4.183
        self.goal1.target_pose.pose.orientation.z = 0.005
        self.goal1.target_pose.pose.orientation.w = 1

        self.goal3.target_pose.pose.position.x = 0.543
        self.goal3.target_pose.pose.position.y = -3.465
        self.goal3.target_pose.pose.orientation.z = 0.016
        self.goal3.target_pose.pose.orientation.w = 1

        self.goal4.target_pose.pose.position.x = 3.254
        self.goal4.target_pose.pose.position.y = -4.327
        self.goal4.target_pose.pose.orientation.z = -0.015
        self.goal4.target_pose.pose.orientation.w = 1

        self.goal5.target_pose.pose.position.x = 3.532
        self.goal5.target_pose.pose.position.y = -4.502
        self.goal5.target_pose.pose.orientation.z = 0.705
        self.goal5.target_pose.pose.orientation.w = 0.710

        self.goal6.target_pose.pose.position.x = 3.551
        self.goal6.target_pose.pose.position.y = -4.234
        self.goal6.target_pose.pose.orientation.z = 0.674
        self.goal6.target_pose.pose.orientation.w = 0.738

        self.goal7.target_pose.pose.position.x = 3.593
        self.goal7.target_pose.pose.position.y = -2.720
        self.goal7.target_pose.pose.orientation.z = 0.707
        self.goal7.target_pose.pose.orientation.w = 0.707

        self.goal8.target_pose.pose.position.x = 1.929
        self.goal8.target_pose.pose.position.y = -2.313
        self.goal8.target_pose.pose.orientation.z = 1
        self.goal8.target_pose.pose.orientation.w = 0.015

        self.goal9.target_pose.pose.position.x = 1.425
        self.goal9.target_pose.pose.position.y = -3.556
        self.goal9.target_pose.pose.orientation.z = -0.705
        self.goal9.target_pose.pose.orientation.w = 0.710

        self.goal_p1 = [
            self.goal3,
        ]
        self.goal_p2 = [
            self.goal4,
            self.goal5,
            self.goal6,
            self.goal7,
        ]
        self.goal_p3 = [
            self.goal8,
        ]
        self.goal_p4 = [
            self.goal9,
        ]
        self.goal_p5 = [
            self.goal1,
        ]

    def image_callback(self, msg):
        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([180, 255, 30])

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        kernel1 = numpy.ones((9, 9), numpy.uint8)

        h, w, d = image.shape

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)

        ori = numpy.array([[8, 233], [312, 233], [78, 180], [245, 180]])
        dst = numpy.array([[80, 240], [220, 240], [90, 80], [220, 80]])

        hinfo, status = cv2.findHomography(ori, dst)
        mask1 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask3 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask4 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask = cv2.warpPerspective(mask, hinfo, (w, h))

        mask1[0:h, w / 2 : w] = 0  # left

        mask4[0:h, 0 : w / 2] = 0  # right

        mask3[h / 2 : h, 0:w] = 0  # upper

        M = cv2.moments(mask)
        M1 = cv2.moments(mask1)
        M3 = cv2.moments(mask3)
        M4 = cv2.moments(mask4)

        # print(self.start)

        # if self.start == 0:
        # self.start += 1
        time.sleep(0.5)
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.4
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(4)
        self.goal_p1[0].target_pose.header.frame_id = "map"
        self.goal_p1[0].target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(self.goal_p1[0])
        str_log = "Send NO. %s Part !!!" % str(1)
        rospy.loginfo(str_log)
        wait = self.client.wait_for_result()
        if wait:
            str_log = "The NO. %s Part achieved success !!!" % str(1)
            rospy.loginfo(str_log)
            time.sleep(1)
            # elif self.start == 1:
        cv2.imshow("window2", mask)
        while M["m00"] > 0 and M3["m00"] > 0:
            print("in")
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            err = w / 2 - cx
            angz = (err * 90.0 / 160) / 9
            self.twist.linear.x = 0.20
            self.twist.angular.z = angz
            self.cmd_vel_pub.publish(self.twist)
            cv2.imshow("window2", mask)
            # if M3["m00"] <= 0:
            # self.start += 1
        print("out")
        time.sleep(10)
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(2)
        self.twist.linear.x = 0
        self.twist.angular.z = -2
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(0.8)
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(1.3)
        self.twist.linear.x = 0.2
        self.twist.angular.z = 1
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(2.5)
        # elif self.start == 2:
        # self.start += 1
        str_log = "Send NO. %s Part !!!" % str(2)
        rospy.loginfo(str_log)
        for i in range(0, 4):
            self.goal_p2[i].target_pose.header.frame_id = "map"
            self.goal_p2[i].target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(self.goal_p2[i])

            str_log = "Send NO. %s Goal !!!" % str(i)
            rospy.loginfo(str_log)

            wait = self.client.wait_for_result()
            if wait:
                str_log = "The NO. %s Goal achieved success !!!" % str(i)
                rospy.loginfo(str_log)
                time.sleep(1)
        str_log = "The NO. %s Part achieved success !!!" % str(2)
        rospy.loginfo(str_log)
        # else:
        print("finished")

        # cv2.imshow("window2", mask)
        cv2.waitKey(1)


rospy.init_node("lane_follower")
follower = Follower()
rospy.spin()
