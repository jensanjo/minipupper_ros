#!/usr/bin/python3
import os
import sys
import time
import rospy
from math import degrees
from trajectory_msgs.msg import JointTrajectory


SERVO_PINS = [15,14,13,  12,11,10,  9,8,7,  6,5,4] #rf lf rb lb
servo_configs = []

class Servo:
    def __init__(self, pin):
        path = f"/sys/class/pwm/pwmchip0/pwm{pin}/duty_cycle"
        self.servo = open(path, 'w')
    
    @staticmethod
    def angle_to_duty_cycle(angle: float):
        return int(round(500_000+1000_000*angle/90))

    def set_duty_cycle(self, duty_cycle: int):
        '''Set the duty cycle for the servo. Range 500_000 .. 2500_000'''
        self.servo.write(str(duty_cycle))
        self.servo.flush()

    def set_angle(self, angle: float):
        self.set_duty_cycle(self.angle_to_duty_cycle(angle))

class ServoInterface:
    def __init__(self):
        self.servos = [Servo(pin) for pin in SERVO_PINS]
        self.count = 0

    def callback(self, data):
        points = data.points[0]
        joint_positions = points.positions
        if self.count == 0:
            print(joint_positions)
        lf1_position = degrees(joint_positions[0])
        lf2_position = degrees(joint_positions[1])
        lf3_position = degrees(joint_positions[2])
        rf1_position = degrees(joint_positions[3])
        rf2_position = degrees(joint_positions[4])
        rf3_position = degrees(joint_positions[5])
        lb1_position = degrees(joint_positions[6])
        lb2_position = degrees(joint_positions[7])
        lb3_position = degrees(joint_positions[8])
        rb1_position = degrees(joint_positions[9])
        rb2_position = degrees(joint_positions[10])
        rb3_position = degrees(joint_positions[11])

        a1 = servo_configs[0]+rf1_position
        a2 = servo_configs[1]-rf2_position
        a3 = servo_configs[2]-90-rf2_position-rf3_position

        a4 = servo_configs[3]+lf1_position
        a5 = servo_configs[4]+lf2_position
        a6 = servo_configs[5]+90+lf2_position+lf3_position

        a7 = servo_configs[6]-rb1_position
        a8 = servo_configs[7]-rb2_position
        a9 = servo_configs[8]-90-rb2_position-rb3_position

        a10 = servo_configs[9]-lb1_position
        a11 = servo_configs[10]+lb2_position
        a12 = servo_configs[11]+90+lb2_position+lb3_position
        
        self.servos[0].set_angle(a1)
        self.servos[1].set_angle(a2)
        self.servos[2].set_angle(a3)

        self.servos[3].set_angle(a4)
        self.servos[4].set_angle(a5)
        self.servos[5].set_angle(a6)

        self.servos[6].set_angle(a7)
        self.servos[7].set_angle(a8)
        self.servos[8].set_angle(a9)

        self.servos[9].set_angle(a10)
        self.servos[10].set_angle(a11)
        self.servos[11].set_angle(a12)

        if self.count == 0:
            print(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12)
        self.count += 1
        
# def set_servo_angle(pin,angle):
# 	duty_cycle = int(500000+11111.11111*angle)
# 	servo_pin = "/sys/class/pwm/pwmchip0/pwm"+str(pin)+"/duty_cycle"
# 	f = open(servo_pin, "w")
# 	f.write(str(int(duty_cycle)))

def get_param():
    global servo_configs
    servo_configs.append(rospy.get_param('rf1_initial_angle'))
    servo_configs.append(rospy.get_param('rf2_initial_angle'))
    servo_configs.append(rospy.get_param('rf3_initial_angle'))

    servo_configs.append(rospy.get_param('lf1_initial_angle'))
    servo_configs.append(rospy.get_param('lf2_initial_angle'))
    servo_configs.append(rospy.get_param('lf3_initial_angle'))

    servo_configs.append(rospy.get_param('rb1_initial_angle'))
    servo_configs.append(rospy.get_param('rb2_initial_angle'))
    servo_configs.append(rospy.get_param('rb3_initial_angle'))

    servo_configs.append(rospy.get_param('lb1_initial_angle'))
    servo_configs.append(rospy.get_param('lb2_initial_angle'))
    servo_configs.append(rospy.get_param('lb3_initial_angle'))

def listener():
    rospy.init_node('servo_interface',anonymous=True)
    get_param()
    servos = ServoInterface()
    rospy.Subscriber("/joint_group_position_controller/command",JointTrajectory,
        servos.callback,queue_size=1)
    rospy.spin()

if __name__=='__main__':
    listener()
