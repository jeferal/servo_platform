#!/usr/bin/env python3

"""
This script will generate a sequence of steps for the ball to follow.
The steps should change every 5 seconds.
"""

from time import sleep

import rospy

from sp_ros_driver.msg import SpCommand


def main():

    rospy.init_node("steps_trajectory")

    # Create publisher
    publisher = rospy.Publisher("/sp_ball_control_node/set_point", SpCommand, queue_size=10)

    step_time = 5.0
    sings = [(1,1), (1,-1), (-1,-1), (-1,1)]

    while not rospy.is_shutdown():
        # Every step_time seconds, change the step
        i = int(rospy.get_time() / step_time) % 4

        x_value = 640 + 150 * sings[i][0]
        y_value = 380 + 150 * sings[i][1]

        # Create message
        msg = SpCommand()
        msg.set_point_roll = int(x_value)
        msg.set_point_pitch = int(y_value)

        # Publish message
        publisher.publish(msg)

        # Sleep
        sleep(0.033)


if __name__ == "__main__":
    main()
