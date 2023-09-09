#include <iostream>

#include "sp_perception/sp_ball_detection_node.h"

using namespace servo_platform;


SpBallDetector::SpBallDetector(ros::NodeHandle& nh) :
    nh_(nh),
    it_(nh)
{
    // Create subscriber
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &SpBallDetector::imageCallback, this);
    
    // Create publisher
    image_pub_ = it_.advertise("/ball_detection", 1);

    // Create publisher for the servo platform command
    sp_command_pub_ = nh_.advertise<sp_ros_driver::SpCommand>("/sp_ball_position", 1);
}

void SpBallDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Detect the ball with a hough transform
    cv::Point2f ball_position;
    try
    {
        ball_position = ball_detector::detect_ball_hough_transform(cv_ptr->image);

        // Draw the ball position
        cv::circle(cv_ptr->image, ball_position, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

        // TODO change for a point message
        sp_ros_driver::SpCommand sp_command_msg;
        sp_command_msg.set_point_roll = ball_position.x;
        sp_command_msg.set_point_pitch = ball_position.y;

        // Publish the position of the ball
        sp_command_pub_.publish(sp_command_msg);
    }
    catch (const std::exception& e)
    {
        ROS_WARN_THROTTLE(1, "Exception: %s", e.what());
    }

    // Draw a red point in the position 300, 300
    cv::circle(cv_ptr->image, cv::Point2f(300, 250), 3, cv::Scalar(0, 0, 255), -1, 8, 0);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sp_ball_detection_node");

    ros::NodeHandle nh("~");

    SpBallDetector sp_ball_detector(nh);

    ros::spin();

    return 0;
}