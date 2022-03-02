#!/usr/bin/python3

from pathlib import Path
import rospy
from sensor_msgs.msg import BatteryState
import std_msgs
import numpy as np

POWER_SUPPLY='max1720x_battery'
POWER_SUPPLY_SYS_PATH = Path('/sys/class/power_supply')
POWER_SUPPLY_DIR = POWER_SUPPLY_SYS_PATH / POWER_SUPPLY
AVERAGE_COUNT = 20  # average over this number of samples
RATE = 1            # messages per second

class BatteryStatePublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('/battery', BatteryState, queue_size=10)
        rospy.init_node('battery_node')
        base = POWER_SUPPLY_DIR
        self.voltage_now_path = base / 'voltage_now'
        self.current_now_path = base / 'current_now'
        self.voltage_ocv_path = base / 'voltage_ocv'

    def run(self):
        rate = rospy.Rate(RATE * AVERAGE_COUNT)
        count = 0
        voltage = []
        current = []
        while not rospy.is_shutdown():
            # print(count)
            if count < AVERAGE_COUNT: 
                voltage_now = 0.001 * int(open(self.voltage_now_path).read())
                current_now = 1e-6 * int(open(self.current_now_path).read())
                voltage.append(voltage_now)
                current.append(current_now)
                count += 1
            else:
                voltage_avg = np.mean(voltage)
                current_avg = np.mean(current)
                count = 0
                voltage = []
                current = []
                voltage_ocv = 1e-3 * int(open(self.voltage_ocv_path).read())

                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                msg = BatteryState(header=header)
                msg.voltage = voltage_now
                msg.current = current_now
                # trick: store the estimated open circuit voltage in the temperature value
                msg.temperature = voltage_ocv
                self.publisher.publish(msg)
            rate.sleep()



if __name__ == '__main__':
    try:
        BatteryStatePublisher().run()
    except rospy.ROSInterruptException:
        pass