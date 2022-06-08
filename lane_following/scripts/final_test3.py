#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber("camera/image", Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.step = 0

    def image_callback(self, msg):
        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([180, 255, 30])

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        kernel1 = numpy.ones((9, 9), numpy.uint8)

        h, w, d = image.shape

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(hsv, lower_black, upper_black)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)

        ori = numpy.array([[8, 233], [312, 233], [78, 180], [245, 180]])
        dst = numpy.array([[80, 240], [220, 240], [90, 80], [220, 80]])

        hinfo, status = cv2.findHomography(ori, dst)
        mask1 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask2 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask3 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask4 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask = cv2.warpPerspective(mask, hinfo, (w, h))

        mask1[0:h, w / 2 : w] = 0  # left

        mask2[0:h, 0 : w / 2] = 0  # right

        mask3[h / 2 : h, 0:w] = 0  # upper

        # mask4[0:h, 0 : w / 2] = 0  # right down
        # mask4[0 : h * 2 / 3, 0:w] = 0

        M = cv2.moments(mask)
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        M3 = cv2.moments(mask3)
        # M4 = cv2.moments(mask4)

        if self.step == 0:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        elif self.step == 2:
            print("step 2")
            self.step = 3
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(2)
            self.twist.linear.x = 0.2
            self.twist.angular.z = -3
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1.5)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 1
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(2.5)
        elif self.step == 4:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        if M["m00"] > 0:
            if M3["m00"] > 0:
                print("step 1")
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                err = w / 2 - 10 - cx
                angz = (err * 90.0 / 160) / 15
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
            # else:
            # self.step = self.step + 1

        cv2.imshow("window2", mask)
        cv2.waitKey(1)


rospy.init_node("lane_follower")
follower = Follower()
rospy.spin()
