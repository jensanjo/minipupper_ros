#!/usr/bin/env python3
from enum import Enum
from pathlib import Path
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import cv2
import cv_bridge

'''
Display icons on mini_pupper display, as used in Quadruped run_robot.py
We subscribe to cmd_vel messages and select an image based on current Twist.
'''

# TODO package with LCD ?
CARTOONS_FOLDER = Path('cartoons')

class State(Enum):
    Init = 0,
    Idle = 1,
    Walking = 2,
    Turning = 3,


class DisplayController:
    def __init__(self):
        rospy.init_node('display_controller', anonymous=True)
        rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, self.callback)
        self.publisher = rospy.Publisher('/display_interface/image',
            sensor_msgs.msg.Image, queue_size=10)
        self.state = State.Idle
        self.bridge = cv_bridge.CvBridge()
        self.image_names = {
            State.Init: 'notconnect.png',
            State.Idle: 'logo.png',
            State.Walking: 'walk_r1.png',
            State.Turning: 'turnaround.png',
        }
        # load cartoons
        assert CARTOONS_FOLDER.is_dir()
        self.images = {}
        for key, image_name in self.image_names.items():
            image_path = CARTOONS_FOLDER / image_name
            image = cv2.imread(str(image_path))
            assert image is not None
            self.images[key] = image
        self.publish_state(self.state)
    
    def publish_state(self, state):
        rospy.loginfo(state.name)
        image = self.images[state]
        msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.publisher.publish(msg)

    def callback(self, msg: geometry_msgs.msg.Twist):
        if msg.angular.z:
            state = State.Turning
        elif msg.linear.x:
            state = State.Walking
        else:
            state = State.Idle
        if state != self.state:
            self.publish_state(state)
        self.state = state

    def run(self):
        '''publish image at least once per second'''
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_state(self.state)
            rate.sleep()

if __name__ == '__main__':
    dc = DisplayController()
    dc.run()
    # rospy.spin()


