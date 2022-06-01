import sys
import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from sound_play.libsoundplay import SoundClient


def callback(msg):
    print("ok")
    rospy.loginfo("I heard id=%d and distance=%f", 3, 1)

    soundhandle = SoundClient()
    rospy.sleep(0.5)
    # if id < 18 and id > 0 and distance < 1.2:
    for k in range(3):
        soundhandle.play(1, 1)
        rospy.sleep(0.5)
    rospy.sleep(5)

    rospy.sleep(0.5)


def listener():
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("sound", SoundRequest, callback)
    # rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.spin()


if __name__ == "__main__":
    listener()
