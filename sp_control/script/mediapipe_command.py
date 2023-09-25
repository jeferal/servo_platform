#!/usr/bin/env python3

"""
    This will get the hand landmarks and will use them to control the position
    of a circle on the screen. The circle will move following a reference point
    through a 2 order differential equation (mass, spring dumpness).
    The reference point will be the center of a pinch grasp between the thumb
    and the index finger.
"""

import cv2
import mediapipe as mp
import numpy as np
import time
import math

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from sp_ros_driver.msg import SpCommand

from utils.filters import EMAFilter

H = 720
W = 1280

# Create the video capture object
cap = cv2.VideoCapture(0)

# Create the mediapipe hand object
mpHands = mp.solutions.hands
hands = mpHands.Hands()

# Create the mediapipe drawing object
mpDraw = mp.solutions.drawing_utils

# Create the EMA model
first_position_x = W / 2
ema_x = EMAFilter(0.15, first_position_x)

first_position_y = H / 2
ema_y = EMAFilter(0.15, first_position_y)

ref_position_x = first_position_x
ref_position_y = first_position_y

# Create publisher
sp_command_pub = rospy.Publisher("/sp_ball_control_node/set_point", SpCommand, queue_size=10)


def image_callback(msg):
    global ref_position_x, ref_position_y, sp_command_pub

    # Convert the image to OpenCV
    cv_image = None
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as exception:
        rospy.logerr(exception)

    # Get the image from the camera
    success, image = cap.read()

    # Convert the image to RGB
    imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Process the image with mediapipe
    results = hands.process(imageRGB)

    print(results)

    is_tracking = False

    # Check if a hand is detected
    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks: # working with each hand
            for id, lm in enumerate(handLms.landmark):
                h_c, w_c, c_c = image.shape
                h_p, w_p, c_p = cv_image.shape
                cx, cy = int(lm.x * w_p), int(lm.y * h_p)

            mpDraw.draw_landmarks(cv_image, handLms, mpHands.HAND_CONNECTIONS)

            # Get the position of the finger tips
            thumb_tip = handLms.landmark[4]
            index_finger_tip = handLms.landmark[8]

            # Get the distance between the finger tips
            distance_ff_th_tip = math.sqrt(
                (thumb_tip.x - index_finger_tip.x)**2 + (thumb_tip.y - index_finger_tip.y)**2)

            # Get the position of the EMA
            position_x = ema_x.get_position()
            position_y = ema_y.get_position()

            # Get the distance between index tip and EMA
            distance_index_ema = math.sqrt(
                (index_finger_tip.x*W - position_x)**2 + (index_finger_tip.y*H - position_y)**2)

            # Get the distance between thumb tip and EMA
            distance_thumb_ema = math.sqrt(
                (thumb_tip.x*W - position_x)**2 + (thumb_tip.y*H - position_y)**2)

            if distance_ff_th_tip < 0.1:
                # Draw a line between the index tip and thumb tip
                cv2.line(cv_image, (int(thumb_tip.x * W), int(thumb_tip.y * H)),
                        (int(index_finger_tip.x * W), int(index_finger_tip.y * H)),
                        (255, 0, 0), 3)
        
            # Print the distances
            if distance_ff_th_tip < 0.1 and distance_index_ema < 100 and distance_thumb_ema < 100:
                is_tracking = True
            if distance_ff_th_tip > 0.1:
                is_tracking = False

    color = ()
    if is_tracking:
        ref_position_x = index_finger_tip.x * W
        ref_position_y = index_finger_tip.y * H
        color = (0, 255, 0)
    else:
        ref_position_x = first_position_x
        ref_position_y = first_position_y
        color = (0, 0, 255)

    ema_x.update(ref_position_x)
    ema_y.update(ref_position_y)

    # Draw the position of the EMA as a little circle
    cv2.circle(cv_image, (int(ema_x.get_position()), int(ema_y.get_position())), 10, color, -1)

    # Create Sp command message
    sp_command = SpCommand()
    sp_command.header.stamp = rospy.Time.now()
    sp_command.set_point_roll = int(ema_x.get_position())
    sp_command.set_point_pitch = int(ema_y.get_position())

    # Publish the command
    sp_command_pub.publish(sp_command)

    cv2.imshow("Output", cv_image)
    cv2.waitKey(1)


def main():
    
    # Create ROS node
    rospy.init_node("mediapipe_command")

    # Create the subscription to the ball camera
    platform_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    is_tracking = False

    rospy.spin()

    # Close the video capture object
    cap.release()

    # Close all the windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
