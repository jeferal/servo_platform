#!/usr/bin/env python3

import math

import rospy
from sp_ros_driver.msg import SpCommand


if __name__ == "__main__":

    rospy.init_node("sin_cos_example")

    # Create publisher
    publisher = rospy.Publisher("/sp_ball_control_node/set_point", SpCommand, queue_size=10)

    period_time = 10.0
    while not rospy.is_shutdown():
        # Generate sinusoidal trajectory 5 sec period, offset 90 deg, amplitude 90 + 30 deg
        x_value = 640 + 150 * math.sin(2 * math.pi * rospy.get_time() / period_time + math.pi / 2)
        y_value = 380 + 150 * math.cos(2 * math.pi * rospy.get_time() / period_time + math.pi / 2)

        # Create message
        msg = SpCommand()
        msg.set_point_roll = int(x_value)
        msg.set_point_pitch = int(y_value)

        print(f"Publishing roll: {msg.set_point_roll}, pitch: {msg.set_point_pitch}")
        # Publish message
        publisher.publish(msg)

        # Sleep
        rospy.sleep(0.033)
