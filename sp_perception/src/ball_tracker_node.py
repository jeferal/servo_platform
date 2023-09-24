#!/usr/bin/env python3

"""
    This script will create a tracking algorithm available in opencv and opencv-contrib to track
    the position of the ball.
    The available tracking algorithms are:
        - BOOSTING
        - MIL
        - KCF
        - TLD
        - MEDIANFLOW
        - GOTURN
        - MOSSE
        - CSRT
"""

from threading import Thread

import numpy as np
import cv2

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sp_perception.msg import SpTrackingOutput

class DaSiamRPN:
    def __init__(self, kernel_cls1_path, kernel_r1_path, model_path, backend_id=0, target_id=0):
        self._model_path = model_path
        self._kernel_cls1_path = kernel_cls1_path
        self._kernel_r1_path = kernel_r1_path
        self._backend_id = backend_id
        self._target_id = target_id

        self._param = cv2.TrackerDaSiamRPN_Params()
        self._param.model = self._model_path
        self._param.kernel_cls1 = self._kernel_cls1_path
        self._param.kernel_r1 = self._kernel_r1_path
        self._param.backend = self._backend_id
        self._param.target = self._target_id
        self._model = cv2.TrackerDaSiamRPN.create(self._param)

    @property
    def name(self):
        return self.__class__.__name__

    def setBackendAndTarget(self, backendId, targetId):
        self._backend_id = backendId
        self._target_id = targetId

        self._param = cv2.TrackerDaSiamRPN_Params()
        self._param.model = self._model_path
        self._param.kernel_cls1 = self._kernel_cls1_path
        self._param.kernel_r1 = self._kernel_r1_path
        self._param.backend = self._backend_id
        self._param.target = self._target_id
        self._model = cv2.TrackerDaSiamRPN.create(self._param)

    def init(self, image, roi):
        self._model.init(image, roi)

    def update(self, image):
        return self._model.update(image)

    def infer(self, image):
        isLocated, bbox = self._model.update(image)
        score = self._model.getTrackingScore()
        return isLocated, bbox, score

class BallTracker:

    def __init__(self) -> None:
        
        rospy.loginfo(f"Opencv version: {cv2.__version__}")

        # Create a subscriber to get the image
        self.bridge = CvBridge()
        self._image_sub = rospy.Subscriber("input/image_raw", Image, self.image_callback)

        # Create a publisher to publish the image with the bounding box
        self._image_pub = rospy.Publisher("~output/image_raw", Image, queue_size=10)

        # Create a publisher to publish the position of the ball
        self._position_pub = rospy.Publisher("~output/position", SpTrackingOutput, queue_size=10)

        # Flag to indicate if the tracking algorithm is initialized
        self._initialized = False
        self._in_progress = False

        # Get the parameter that indicates the tracking algorithm
        self._tracking_algorithm = rospy.get_param("~tracking_algorithm", "DASIAMRPN")

        # Create the tracker
        self._tracker = None

        # Thread to ask the user for a bounding box
        self._initialize_tracker_thread = Thread(target=self.initialize_tracker, args=())

    def image_callback(self, msg: Image):
        # Convert the image to OpenCV
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exception:
            rospy.logerr(exception)

        # Initialize the tracker if it is not initialized
        if not self._initialized and not self._in_progress:
            self._in_progress = True
            Thread(target=self.initialize_tracker, args=(cv_image,)).start()
            return
        
        if not self._initialized and self._in_progress:
            return

        # Update the tracker
        ok, bbox = self._tracker.update(cv_image)

        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(cv_image, p1, p2, (255, 0, 0), 2, 1)

            # Publish the position of the ball
            output_msg = SpTrackingOutput()
            output_msg.position_x = int(bbox[0] + bbox[2] / 2)
            output_msg.position_y = int(bbox[1] + bbox[3] / 2)
            output_msg.bounding_box[0] = int(bbox[0])
            output_msg.bounding_box[1] = int(bbox[1])
            output_msg.bounding_box[2] = int(bbox[2])
            output_msg.bounding_box[3] = int(bbox[3])

            # Update the time stamp
            output_msg.header.stamp = rospy.Time.now()

            self._position_pub.publish(output_msg)

        else:
            # Tracking failure
            cv2.putText(cv_image,
                        "Tracking failure detected",
                        (100, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, (0, 0, 255), 2)

            # Destroy the tracker
            self._tracker = None

            # Set the flag to indicate that the tracker is not initialized
            self._initialized = False
        
        # Publish the image with the bounding box
        try:
            self._image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as exception:
            rospy.logerr(exception)

    @staticmethod
    def run():
        rospy.spin()
    
    def initialize_tracker(self, image: np.ndarray):
        self.create_tracker(self._tracking_algorithm)

        # Draw a point in the image given x and y
        x = 640
        y = 360
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)

        # Ask the user to create a bounding box
        bbox = cv2.selectROI("Initialization window", image, fromCenter=False, showCrosshair=True)
        # Destroy the window
        cv2.destroyWindow("Initialization window")

        if bbox == (0, 0, 0, 0):
            self._in_progress = False
            return

        # Initialize the tracker with the bounding box
        self._tracker.init(image, bbox)

        # Set the flag to indicate that the tracker is initialized
        self._initialized = True
        self._in_progress = False
    
    def create_tracker(self, tracking_algorithm: str):
        rospy.loginfo(f"Creating tracker of type {tracking_algorithm}")
        if tracking_algorithm == 'BOOSTING':
            self._tracker = cv2.TrackerBoosting_create()
        elif tracking_algorithm == 'MIL':
            self._tracker = cv2.TrackerMIL_create()
        elif tracking_algorithm == 'KCF':
            self._tracker = cv2.TrackerKCF_create()
        elif tracking_algorithm == 'TLD':
            self._tracker = cv2.TrackerTLD_create()
        elif tracking_algorithm == 'MEDIANFLOW':
            self._tracker = cv2.TrackerMedianFlow_create()
        elif tracking_algorithm == 'CSRT':
            self._tracker = cv2.TrackerCSRT_create()
        elif tracking_algorithm == 'MOSSE':
            self._tracker = cv2.TrackerMOSSE_create()
        elif tracking_algorithm == 'DASIAMRPN':
            self._tracker = DaSiamRPN(
                kernel_cls1_path="/opt/opencv_zoo/models/object_tracking_dasiamrpn/object_tracking_dasiamrpn_kernel_cls1_2021nov.onnx",
                kernel_r1_path="/opt/opencv_zoo/models/object_tracking_dasiamrpn/object_tracking_dasiamrpn_kernel_r1_2021nov.onnx",
                model_path="/opt/opencv_zoo/models/object_tracking_dasiamrpn/object_tracking_dasiamrpn_model_2021nov.onnx",
                backend_id=cv2.dnn.DNN_BACKEND_CUDA,
                target_id=cv2.dnn.DNN_TARGET_CUDA)
        else:
            raise ValueError(f"Unknown tracking algorithm {tracking_algorithm}")


def main():
    rospy.init_node("ball_tracker_node", anonymous=True)
    ball_tracker = BallTracker()
    ball_tracker.run()


if __name__ == "__main__":
    main()
