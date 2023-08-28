#!/usr/bin/env python3

import math
from time import sleep

import rospy
from sp_ros_driver.msg import SpCommand


if __name__ == "__main__":

    rospy.init_node("sin_cos_example")

    # Create publisher
    publisher = rospy.Publisher("/sp_ros_driver_node/sp_command", SpCommand, queue_size=10)

    while not rospy.is_shutdown():
        # Generate sinusoidal trajectory 5 sec period, offset 90 deg, amplitude 90 + 30 deg
        period_time = 1.5
        roll_value = 90 + 30 * math.sin(2 * math.pi * rospy.get_time() / period_time + math.pi / 2)
        pitch_value = 90 + 30 * math.cos(2 * math.pi * rospy.get_time() / period_time + math.pi / 2)

        # Create message
        msg = SpCommand()
        msg.roll = int(roll_value)
        msg.pitch = int(pitch_value)
        
        print(f"Publishing roll: {msg.roll}, pitch: {msg.pitch}")
        # Publish message
        publisher.publish(msg)

        sleep(0.05)
