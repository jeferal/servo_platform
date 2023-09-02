#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ball_detector
{

    /// @brief 
    /// @param image 
    /// @return 
    auto detect_ball_hough_transform(cv::Mat image) -> cv::Point2f;

}   // namespace servo_platform
