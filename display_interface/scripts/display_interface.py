#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import Image as Image_ROS
import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import sys
import time
from PIL import Image

from LCD.ST7789 import ST7789

def callback(data):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(data,"bgr8")
    image = Image.fromarray(cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB))
    resized = image.resize((320,240))
    disp.display(resized)

def showImage():
    rospy.init_node('display_interface',anonymous = True)
    rospy.Subscriber('usb_cam/image_raw', Image_ROS, callback)
    rospy.spin()

if __name__ == '__main__':
    disp = ST7789()
    disp.begin()
    disp.clear()
    showImage()
