#include "sp_ros_driver/sp_ros_driver.h"

using namespace servo_platform;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sp_ros_driver_node");

    ros::NodeHandle nh("~");

    // Get port name
    std::string port_name;
    if (!nh.getParam("port_name", port_name))
    {
        ROS_ERROR("Failed to get port name");
        return -1;
    }

    // ROS parameter start roll
    int start_roll;
    if (!nh.getParam("start_roll", start_roll))
    {
        ROS_ERROR("Failed to get start roll");
        return -1;
    }

    // ROS parameter start pitch
    int start_pitch;
    if (!nh.getParam("start_pitch", start_pitch))
    {
        ROS_ERROR("Failed to get start pitch");
        return -1;
    }

    // Create driver
    SpRosDriver sp_ros_driver(port_name.c_str(), nh, start_roll, start_pitch);

    // Initialize driver
    if (!sp_ros_driver.init())
    {
        ROS_ERROR("Failed to initialize driver");
        return -1;
    }

    // Run driver
    sp_ros_driver.run();

    return 0;
}
