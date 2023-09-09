#include <sp_ros_driver/sp_ros_driver.h>

using namespace servo_platform;

SpRosDriver::SpRosDriver(const char* port_name, ros::NodeHandle& nh, int start_roll, int start_pitch) :
_nh(nh),
SpDriver(port_name)
{
    _current_roll = start_roll;
    _current_pitch = start_pitch;
}

bool SpRosDriver::init()
{
    // Initialize driver
    if (SpDriver::init() != 0)
    {
        ROS_ERROR("Failed to initialize driver");
        return false;
    }

    // Create publisher
    _state_pub = _nh.advertise<sp_ros_driver::SpState>("state", 1);

    // Create subscriber
    _command_sub = _nh.subscribe("command", 1, &SpRosDriver::commandCb, this);

    spinner_.start();

    return true;
}

void SpRosDriver::commandCb(const sp_ros_driver::SpCommand& sp_command_msg)
{
    // Update current roll and pitch
    _current_roll = sp_command_msg.set_point_roll;
    _current_pitch = sp_command_msg.set_point_pitch;
}

void SpRosDriver::run()
{
    while (ros::ok())
    {
        // Read roll and pitch
        int roll = _current_roll;
        int pitch = _current_pitch;
        step(roll, pitch, _state);

        // Publish state
        sp_ros_driver::SpState sp_state_msg;
        sp_state_msg.actual_roll = _state.state.roll;
        sp_state_msg.actual_pitch = _state.state.pitch;

        sp_state_msg.desired_roll = roll;
        sp_state_msg.desired_pitch = pitch;

        // Add the time stamp
        sp_state_msg.header.stamp = ros::Time::now();

        _state_pub.publish(sp_state_msg);
    }
}
