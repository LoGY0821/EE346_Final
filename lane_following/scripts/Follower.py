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
        self.stopped = False
        self.init = 0
        self.start = 0
        self.right = 0
        self.starttimer = time.time()
        self.finish = 0

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

        mask4[0:h, 0 : w / 2] = 0  # right

        mask3[0 : h / 3, 0:w] = 0  # right middle
        mask3[2 * h / 3 : h, 0:w] = 0  #
        mask3[0:h, 0 : w / 2] = 0

        mask2[0:h, 0 : w / 2] = 0  # right down
        mask2[0 : h * 2 / 3, 0:w] = 0

        M = cv2.moments(mask)
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        M3 = cv2.moments(mask3)
        M4 = cv2.moments(mask4)

        if M["m00"] > 0:
            if self.start == 0:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(3.5)
                self.twist.linear.x = 0.2
                self.twist.angular.z = -3
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(1)
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(2.25)
                self.twist.linear.x = 0.2
                self.twist.angular.z = 2.5
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(1)
                self.start = 1
            thistime = time.time()
            if (
                thistime - self.starttimer >= 75
                and self.right == 0
                and M3["m00"] <= 0
                and self.finish == 0
            ):
                self.right = 1
            if self.right == 1:
                print("detecting right")
                if M2["m00"] <= 0 and M1["m00"] > 0:
                    cx1 = int(M["m10"] / M["m00"])
                    cy1 = int(M["m01"] / M["m00"])
                    if cy1 <= 170:
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                        time.sleep(1.5)
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -3
                        self.cmd_vel_pub.publish(self.twist)
                        self.right = 0
                        self.finish = 1
                        time.sleep(1.25)
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                        time.sleep(0.25)

            cx1 = int(M["m10"] / M["m00"])
            cy1 = int(M["m01"] / M["m00"])
            if M1["m00"] > 0:
                cx2 = int(M1["m10"] / M1["m00"])
                cy2 = int(M1["m01"] / M1["m00"])
                cv2.circle(mask, (cx2, cy2), 5, (255, 255, 255), -1)
            else:
                cx2 = 240
                cy2 = 120
            if M4["m00"] > 0:
                cx3 = int(M4["m10"] / M4["m00"])
                cy3 = int(M4["m01"] / M4["m00"])
                cv2.circle(mask, (cx3, cy3), 5, (255, 255, 255), -1)
            else:
                cx3 = 80
                cy3 = 120
            cv2.circle(mask, (cx1, cy1), 10, (255, 255, 255), -1)
            fx = (cx2 + cx3) / 2
            fy = (cy2 + cy3) / 2
            cv2.circle(mask, (fx, fy), 5, (255, 255, 255), -1)
            err = w / 2 - 10 - fx
            angz = (err * 90.0 / 160) / 9

            self.twist.linear.x = 0.20
            self.twist.angular.z = angz
            self.cmd_vel_pub.publish(self.twist)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            param = aruco.DetectorParameters_create()

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, markerID, rejected = aruco.detectMarkers(
                gray, aruco_dict, parameters=param
            )
            matrix = numpy.array(
                [
                    [251.90279, 0.0, 162.34701],
                    [0.0, 251.36665, 127.8276],
                    [0.0, 0.0, 1.0],
                ]
            )
            dist = numpy.array([[0.162913, -0.278889, -0.001218, 0.000077, 0.000000]])

            if len(corners) > 0 and not self.stopped:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, matrix, dist
                )
                (rvec - tvec).any()
                for i in range(rvec.shape[0]):
                    aruco.drawDetectedMarkers(image, corners, markerID)
                distance = int(tvec[0][0][2] * 1000)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                print("distance: ", distance, "mm")
                if distance <= 600:
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    time.sleep(3)
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    self.stopped = True
                    time.sleep(30)

        cv2.imshow("window2", mask)
        cv2.waitKey(1)


rospy.init_node("lane_follower")
follower = Follower()
rospy.spin()
