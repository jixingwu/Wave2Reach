#!/usr/bin/env python
import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String


class OpenPose:
    def __init__(self):
        self.image_pub = rospy.Publisher('image_raw', Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('openpose', String, self.callback)

    def callback(self, data):
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        openpose(cv_image)


def openpose(img):
    cv2.imshow('image_raw', img)
    cv2.waitKey(2)


def main(args):
    OP = OpenPose()
    rospy.init_node('OpenPose', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
