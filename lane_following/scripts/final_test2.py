#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import pyttsx3


class Follower:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber("camera/image", Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.timer = time.time()
        self.step = 0
        self.havebeep = 0

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
        mask5 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask6 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask = cv2.warpPerspective(mask, hinfo, (w, h))

        mask1[0:h, w / 2 : w] = 0  # left

        mask2[0:h, 0 : w / 2] = 0  # right

        mask3[h / 3 : h, 0:w] = 0  # upper

        mask4[40:h, 0:w] = 0  # left up
        mask4[0:h, w / 2 : w] = 0
        # mask4[0:20, 0:w] = 0

        mask5[40:h, 0:w] = 0  # right up
        mask5[0:h, 0 : w / 2] = 0
        # mask5[0:20, 0:w] = 0
        mask6[2 * h / 3 : h, 0:w] = 0  # upper

        M = cv2.moments(mask)
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        M3 = cv2.moments(mask3)
        M4 = cv2.moments(mask4)
        M5 = cv2.moments(mask5)
        M6 = cv2.moments(mask6)

        if self.step == 0:
            time.sleep(0.5)
            print("start")
            self.step = 1
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(5)
            self.twist.linear.x = 0.2
            self.twist.angular.z = -2
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.8)
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.2)
        elif self.step == 2:
            print("go into trace")
            self.step = 3
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(2.2)
            self.twist.linear.x = 0.2
            self.twist.angular.z = -3
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1.3)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 1
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(2.5)
            self.timer = time.time()
        elif self.step == 4:
            print("find p2")
            self.step = 5
            self.twist.linear.x = 0.2
            self.twist.angular.z = -0.3
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(3.7)
            self.twist.linear.x = 0
            self.twist.angular.z = 3
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.8)
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.5)
            self.timer = time.time()
        elif self.step == 6:
            print("exit turn")
            self.step = 7
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.1
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.5)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(3)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 1
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(2.85)
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(4.9)
            print("go home")
            self.twist.linear.x = 0
            self.twist.angular.z = -2
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.9)
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(3)
            self.timer = time.time()

        if M["m00"] > 0:
            thistime = time.time()
            if (
                M4["m00"] <= 0
                and M5["m00"] > 0
                and self.step == 3
                and thistime - self.timer > 1
            ):
                self.step = 4
            elif (
                M4["m00"] > 0
                and M5["m00"] <= 0
                and self.step == 5
                and thistime - self.timer > 15
            ):
                self.step = 6
            elif M3["m00"] <= 0 and self.step == 1 and thistime - self.timer > 1:
                self.step = 2
            elif M6["m00"] <= 0 and self.step == 7 and thistime - self.timer > 0.5:
                print("find p1")
                self.step = 1
                self.havebeep = 0
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(4)
                self.twist.linear.x = 0.2
                self.twist.angular.z = 2
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(0.9)
                # time.sleep(0.8)
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(5.25)
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(0.5)
                self.twist.linear.x = -0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(5)
                self.twist.linear.x = 0
                self.twist.angular.z = 3
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(1)
                self.timer = time.time()
            else:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                err = w / 2 - 10 - cx
                angz = (err * 90.0 / 160) / 18
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

        if len(corners) > 0 and self.havebeep == 0:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, matrix, dist)
            (rvec - tvec).any()
            print("[INFO] ArUco marker ID: {}".format(markerID))
            engine = pyttsx3.init()
            # engine.setProperty("rate", 200)
            # volume = engine.getProperty("volume")
            # engine.setProperty("volume", 1)
            # voices = engine.getProperty("voices")
            # engine.setProperty("voice", voices[0].id)
            engine.say(str(markerID))
            engine.runAndWait()
            self.havebeep = 1

        cv2.imshow("window2", mask)
        cv2.waitKey(1)


rospy.init_node("lane_follower")
follower = Follower()
rospy.spin()
