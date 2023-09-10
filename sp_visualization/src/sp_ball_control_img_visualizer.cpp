#include "sp_visualization/sp_ball_control_img_visualizer.h"

using namespace sp_visualization;


SpBallControlImgVisualizer::SpBallControlImgVisualizer(ros::NodeHandle& nh) :
nh_(nh),
it_(nh),
ball_x_(-1), ball_y_(-1),
set_point_x_(640), set_point_y_(360)    // TODO: Change when implementing state topic in controller
{
}

bool SpBallControlImgVisualizer::init()
{
    // Create subscription to image
    image_sub_ = it_.subscribe("image_in", 1, &SpBallControlImgVisualizer::imageCallback, this);

    // Create subscription to ball detection
    ball_detection_sub_ = nh_.subscribe("ball_detection", 1, &SpBallControlImgVisualizer::ballDetectionCallback, this);

    // Create subscription to set point
    // TODO: Change when implementing state topic in controller
    set_point_sub_ = nh_.subscribe("set_point", 1, &SpBallControlImgVisualizer::setPointCallback, this);

    // Create publisher of the output image
    image_pub_ = it_.advertise("image_out", 1);

    return true;
}

void SpBallControlImgVisualizer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the image to OpenCV format
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw all the set points
    for (auto& point : points_vector_)
    {
        cv::circle(cv_ptr->image, cv::Point(point.set_point_roll, point.set_point_pitch), 5, CV_RGB(255,0,0), 2);
    }

    // Remove an element from the vector if the size is bigger than N
    if (points_vector_.size() > 10)
    {
        points_vector_.erase(points_vector_.begin());
    }

    // Draw the ball position and the bounding box
    if (ball_x_ != -1 && ball_y_ != -1)
    {
        cv::circle(cv_ptr->image, cv::Point(ball_x_, ball_y_), 5, CV_RGB(0,255,0), 2);
        cv::rectangle(cv_ptr->image, cv::Point(bounding_box_[0], bounding_box_[1]), cv::Point(bounding_box_[0] + bounding_box_[2], bounding_box_[1] + bounding_box_[3]), CV_RGB(0,255,0), 2);
    }

    // Publish the image
    image_pub_.publish(cv_ptr->toImageMsg());
}

void SpBallControlImgVisualizer::ballDetectionCallback(const sp_perception::SpTrackingOutput::ConstPtr& msg)
{
    // Update ball position
    ball_x_ = msg->position_x;
    ball_y_ = msg->position_y;

    // Update the bouding box
    for (int i = 0; i < 4; i++)
    {
        bounding_box_[i] = msg->bounding_box[i];
    }
}

void SpBallControlImgVisualizer::setPointCallback(const sp_control::BallControlPid::ConstPtr& msg)
{
    static int counter = 0;

    // TODO: Create a temporal throttle to avoid printing too many points
    if (counter == 10)
    {
        // Create message
        sp_ros_driver::SpCommand set_point_msg;
        set_point_msg.set_point_roll = msg->set_point_x;
        set_point_msg.set_point_pitch = msg->set_point_y;

        // Add message to the queue
        points_vector_.emplace_back(set_point_msg);

        counter = 0;
        return;
    }

    counter++;
    return;
}

void SpBallControlImgVisualizer::run()
{
    // Spin
    ros::spin();
}

int main(int argc, char** argv)
{

    // Init node
    ros::init(argc, argv, "sp_ball_control_img_visualizer");

    // Create node handle
    ros::NodeHandle nh("~");

    // Create visualizer
    SpBallControlImgVisualizer visualizer(nh);

    // Init visualizer
    visualizer.init();

    // Run visualizer
    visualizer.run();

    return 0;
}