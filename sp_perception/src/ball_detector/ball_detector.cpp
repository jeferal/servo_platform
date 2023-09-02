#include <sp_perception/ball_detector/ball_detector.h>


auto ball_detector::detect_ball_hough_transform(cv::Mat image) -> cv::Point2f
{
    // Convert to grayscale
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur
    cv::GaussianBlur(image, image, cv::Size(5, 5), 10);

    // Apply Hough transform
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT, 1, image.rows/4, 100, 30, 25, 70);

    // If no circles were found, throw an exception
    if (circles.size() == 0)
    {
        throw std::runtime_error("No circles were found");
    }

    // Return only the first circle
    return cv::Point2f(circles[0][0], circles[0][1]);
}
