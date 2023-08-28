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

    // Create driver
    SpRosDriver sp_ros_driver(port_name.c_str(), nh, sleep_ms);

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
