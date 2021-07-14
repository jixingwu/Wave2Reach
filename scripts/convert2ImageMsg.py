#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from Wave2Reach.msg import ImageMsg#, DepthMsg

class image_listenner:
    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/sensors/stereo_cam/left/image_rect_color',Image,self.image_sub_callback)
        # self.depth_sub = rospy.Subscriber('/sensors/stereo_cam/left/image_rect_color', Image, self.depth_sub_callback)
        self.image_pub = rospy.Publisher('/image_data', ImageMsg, queue_size=1)
        self.depth_pub = rospy.Publisher('/depth_data', ImageMsg, queue_size=1)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            size = self.img.shape
            image = ImageMsg()
            image.height = size[0] # 480
            image.width = size[1] # 640
            image.channels = size[2] # 3
            # image.data = data.data # image_data
            image.data = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8").data
            self.image_pub.publish(image)
        except CvBridgeError as e:
            print(e) 

    def depth_sub_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            size = depth_image.shape
            # print(size)
            depth = ImageMsg()
            depth.height = size[0]
            depth.width = size[1]
            depth.channels = 1
            depth.data = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1").data
            self.depth_pub.publish(depth)
        except CvBridgeError as e:
            print(e)



if __name__ == '__main__':
    print('Starting convert2ImageMsg node')
    image_listenner = image_listenner()
    rospy.init_node('convert2ImageMsg', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    cv2.destroyAllWindows()