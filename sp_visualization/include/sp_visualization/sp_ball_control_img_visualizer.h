#include <array>

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sp_ros_driver/SpCommand.h>
#include <sp_perception/SpTrackingOutput.h>


namespace sp_visualization
{

    class SpBallControlImgVisualizer
    {

        public:
            /// @brief 
            SpBallControlImgVisualizer(ros::NodeHandle& nh);

            /// @brief 
            /// @return 
            bool init();

            /// @brief 
            /// @param msg 
            void imageCallback(const sensor_msgs::ImageConstPtr& msg);

            /// @brief 
            /// @param msg 
            void ballDetectionCallback(const sp_perception::SpTrackingOutput::ConstPtr& msg);

            /// @brief 
            /// @param msg 
            void setPointCallback(const sp_ros_driver::SpCommand::ConstPtr& msg);

            void run();

        private:

            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            image_transport::Publisher image_pub_;            

            // Subscription to ball detection
            ros::Subscriber ball_detection_sub_;
            // Subscription to set point
            ros::Subscriber set_point_sub_;

            // Ball position
            int ball_x_;
            int ball_y_;

            // Bounding box
            std::array<int, 4> bounding_box_ = {-1, -1, -1, -1};

            // Set point
            int set_point_x_;
            int set_point_y_;

            cv_bridge::CvImagePtr cv_ptr;
            std::vector<sp_ros_driver::SpCommand> points_vector_;
    };

}  // namespace sp_visualization