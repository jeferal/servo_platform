#!/usr/bin/env python3

import pygame

import rospy
from sp_ros_driver.msg import SpCommand


if __name__ == "__main__":

    # Init node
    rospy.init_node("xbox_controller")

    # Create publisher
    publisher = rospy.Publisher("/sp_ros_driver_node/sp_command", SpCommand, queue_size=10)

    # Get start roll parameter
    start_roll = rospy.get_param("~start_roll", 90)
    # Get start pitch parameter
    start_pitch = rospy.get_param("~start_pitch", 90)

    x_value_ = start_roll
    y_value_ = start_pitch

    pygame.init()
    joysticks = []
    clock = pygame.time.Clock()
    keepPlaying = True

    # for al the connected joysticks
    for i in range(0, pygame.joystick.get_count()):
        # create an Joystick object in our list
        joysticks.append(pygame.joystick.Joystick(i))
        # initialize the appended joystick (-1 means last array item)
        joysticks[-1].init()
        # print a statement telling what the name of the controller is
        print(f"Detected joystick {joysticks[-1].get_name()}")

    while not rospy.is_shutdown():
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == 1536:
                if event.dict["axis"] == 0:
                    event_value = event.dict["value"]
                    x_value_ = int(- 30 * event_value + start_roll)
                elif event.dict["axis"] == 1:
                    event_value = event.dict["value"]
                    y_value_ = int(30 * event_value + start_pitch)
                else:
                    continue
                # Crop the values
                if x_value_ > 150:
                    x_value_ = 150
                elif x_value_ < 30:
                    x_value_ = 30
                if y_value_ > 150:
                    y_value_ = 150
                elif y_value_ < 30:
                    y_value_ = 30
                
                # Create message
                msg = SpCommand()
                msg.roll = y_value_
                msg.pitch = x_value_

                print(f"Publishing roll: {msg.roll}, pitch: {msg.pitch}")
                # Publish message
                publisher.publish(msg)
