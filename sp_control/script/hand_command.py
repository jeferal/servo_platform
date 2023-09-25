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


H = 720
W = 1280

class MSDModel:
    def __init__(self, mass, spring, dumpness, first_position=0, first_velocity=0, first_acceleration=0, min_position=0, max_position=1):
        self.mass = mass
        self.spring = spring
        self.dumpness = dumpness
        self.position = first_position
        self.velocity = first_velocity
        self.acceleration = first_acceleration

        self._min_position = min_position
        self._max_position = max_position

    def update(self, reference) -> None:
        accel_value = (self.spring * (reference - self.position) +
                             self.dumpness * self.velocity) / self.mass
        vel_value = self.velocity + self.acceleration
        pos_value = self.position + self.velocity

        self.position = self._saturate(pos_value, self._min_position, self._max_position)
        self.velocity = self._saturate(vel_value, -10, 10)
        self.acceleration = self._saturate(accel_value, -10, 10)
    
        self.velocity += self.acceleration
        self.position += self.velocity
    
    def _saturate(self, value, min_value, max_value) -> float:
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value

    def get_position(self) -> float:
        return self.position
    
    def get_velocity(self) -> float:
        return self.velocity
    
    def get_acceleration(self) -> float:
        return self.acceleration


def main():
    
    # Create the video capture object
    cap = cv2.VideoCapture(0)

    # Create the mediapipe hand object
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()

    # Create the mediapipe drawing object
    mpDraw = mp.solutions.drawing_utils

    # Create the MSD model
    mass_x = 0.0
    spring_x = 1.0
    dumpness_x = 5.0
    first_position_x = W / 2
    msd_x = MSDModel(mass_x, spring_x, dumpness_x, first_position_x, min_position=0, max_position=W)

    mass_y = 0.0
    spring_y = 1.0
    dumpness_y = 5.0
    first_position_y = H / 2
    msd_y = MSDModel(mass_y, spring_y, dumpness_y, first_position_y, min_position=0, max_position=H)

    ref_position_x = first_position_x
    ref_position_y = first_position_y

    is_tracking = False

    # Loop
    while True:
        # Get the image from the camera
        success, image = cap.read()

        # Convert the image to RGB
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the image with mediapipe
        results = hands.process(imageRGB)

        # Check if a hand is detected
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks: # working with each hand
                for id, lm in enumerate(handLms.landmark):
                    h, w, c = image.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)

                mpDraw.draw_landmarks(image, handLms, mpHands.HAND_CONNECTIONS)

                # Get the position of the finger tips
                thumb_tip = handLms.landmark[4]
                index_finger_tip = handLms.landmark[8]

                # Get the distance between the finger tips
                distance_ff_th_tip = math.sqrt(
                    (thumb_tip.x - index_finger_tip.x)**2 + (thumb_tip.y - index_finger_tip.y)**2)

                # Get the position of the MSD
                position_x = msd_x.get_position()
                position_y = msd_y.get_position()

                # Get the distance between index tip and MSD
                distance_index_msd = math.sqrt(
                    (index_finger_tip.x*W - position_x)**2 + (index_finger_tip.y*H - position_y)**2)

                # Get the distance between thumb tip and MSD
                distance_thumb_msd = math.sqrt(
                    (thumb_tip.x*W - position_x)**2 + (thumb_tip.y*H - position_y)**2)

                if distance_ff_th_tip < 0.1:
                    # Draw a line between the index tip and thumb tip
                    cv2.line(image, (int(thumb_tip.x * W), int(thumb_tip.y * H)),
                            (int(index_finger_tip.x * W), int(index_finger_tip.y * H)),
                            (255, 0, 0), 3)
                if distance_ff_th_tip < 0.1 and distance_index_msd < 50 and distance_thumb_msd < 50:
                    is_tracking = True
                if distance_ff_th_tip > 0.1:
                    is_tracking = False     
   
        if is_tracking:
            ref_position_x = index_finger_tip.x * W
            ref_position_y = index_finger_tip.y * H
        else:
            ref_position_x = first_position_x
            ref_position_y = first_position_y

        print(f"Is tracking: {is_tracking}")
        msd_x.update(ref_position_x)
        msd_y.update(ref_position_y)
        
        # Draw the position of the MSD as a little circle
        cv2.circle(image, (int(msd_x.get_position()), int(msd_y.get_position())), 10, (0, 0, 255), -1)
        
        cv2.imshow("Output", image)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
