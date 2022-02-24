#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class ImageFlip:
    def __init__(self):
        rospy.init_node('image_flip')
        rospy.Subscriber('cv_camera/image_raw', Image, self.callback)
        self.publisher = rospy.Publisher('/image_flip/image_raw/compressed',
            CompressedImage, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        rotated = cv2.rotate(img, cv2.ROTATE_180)
        # self.data = data
        self.publisher.publish(self.bridge.cv2_to_compressed_imgmsg(rotated))


if __name__ == '__main__':
    node = ImageFlip()
    rospy.spin()
