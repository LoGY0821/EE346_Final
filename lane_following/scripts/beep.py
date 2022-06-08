# -*- coding: UTF-8 -*-
import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import pyttsx3


def beep(msg):
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # 设置发音速率，默认值为200
    engine.setProperty("rate", 200)
    # 设置发音大小，范围为0.0-1.0
    volume = engine.getProperty("volume")
    engine.setProperty("volume", 1)
    # 设置默认的声音：voices[0].id代表男生，voices[1].id代表女生
    voices = engine.getProperty("voices")
    engine.setProperty("voice", voices[0].id)
    # 添加朗读文本
    engine.say("1")

    # 等待语音播报完毕
    engine.runAndWait()
    cv2.imshow("window2", image)
    cv2.waitKey(1)


bridge = cv_bridge.CvBridge()
image_sub = rospy.Subscriber("camera/image", Image, beep)
engine = pyttsx3.init()
