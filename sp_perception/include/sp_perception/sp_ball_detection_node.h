#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sp_ros_driver/SpCommand.h>

#include "ball_detector/ball_detector.h"


namespace servo_platform
{

    class SpBallDetector
    {

        public:
            /// @brief 
            SpBallDetector(ros::NodeHandle& nh);

            /// @brief 
            /// @param msg 
            void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        private:

            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            image_transport::Publisher image_pub_;

            // Publisher for the servo platform command
            ros::Publisher sp_command_pub_;
    };

}  // namespace servo_platform
