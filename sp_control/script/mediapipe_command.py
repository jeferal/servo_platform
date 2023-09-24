#!/usr/bin/env python3

"""
    This script will be used to send commands to the ball position controller through a mediapipe model.
    The script will read the position of the first finger tip and this position will be the goal to the ball position controller
    as long as the position of the ff tip remains within the frame and within the playable area.
"""

import cv2
import mediapipe as mp
import numpy as np

import rospy

from sp_ros_driver.msg import SpCommand

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Create a VideoCapture object to capture webcam feed
cap = cv2.VideoCapture(0)

# Init node
rospy.init_node("mediapipe_command")

# Create the publisher
pub = rospy.Publisher("/sp_ball_control_node/set_point", SpCommand, queue_size=1)

x_set_point = 640
y_set_point = 380

while not rospy.is_shutdown():

    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert the frame to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect hand landmarks
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:

        # Get the position of the first finger tip in the camera frame
        landmarks = results.multi_hand_landmarks[0].landmark
        index_tip = landmarks[8]
        index_x, index_y = index_tip.x, index_tip.y
        # Convert to camera coordinates with the shape of the image
        x_set_point = int(index_x * frame.shape[1])
        y_set_point = int(index_y * frame.shape[0])

        # Draw the landmarks
        for landmark in landmarks:
            x = int(landmark.x * frame.shape[1])
            y = int(landmark.y * frame.shape[0])
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    # Create the message
    msg = SpCommand()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "mediapipe_command"
    msg.set_point_roll = x_set_point
    msg.set_point_pitch = y_set_point

    print(f"Publishing set point: {x_set_point}, {y_set_point}")

    # Publish the message
    pub.publish(msg)

    # Display the frame
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
        break

cap.release()
cv2.destroyAllWindows()
