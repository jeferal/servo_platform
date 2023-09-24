import cv2
import mediapipe as mp
import numpy as np

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Create a VideoCapture object to capture webcam feed
cap = cv2.VideoCapture(0)

# Initialize circle parameters
circle_radius = 30
circle_color = (0, 0, 255)  # Red
circle_center = np.array([300.0, 300.0])  # Initial position
mass = 1.0
spring_constant = 0.1
damping_coefficient = 0.02

# Initialize variables for velocity and acceleration
velocity = np.array([0.0, 0.0])
acceleration = np.array([0.0, 0.0])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect hand landmarks
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        landmarks = results.multi_hand_landmarks[0].landmark
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]

        # Calculate the distance between thumb tip and index tip
        thumb_x, thumb_y = thumb_tip.x, thumb_tip.y
        index_x, index_y = index_tip.x, index_tip.y
        distance = np.sqrt((thumb_x - index_x) ** 2 + (thumb_y - index_y) ** 2)

        # Apply the second-order differential equation to update circle position
        force = -spring_constant * distance - damping_coefficient * velocity
        acceleration = force / mass
        velocity += acceleration
        circle_center += velocity
    else:
        # If no hand is detected, apply spring force towards the center
        center_force = -spring_constant * (circle_center - np.array([300.0, 300.0]))
        acceleration = center_force / mass
        velocity += acceleration
        circle_center += velocity

    # Draw the circle
    cv2.circle(frame, (int(circle_center[0]), int(circle_center[1])), circle_radius, circle_color, -1)

    # Display the frame
    cv2.imshow('Hand Control', frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
        break

cap.release()
cv2.destroyAllWindows()
