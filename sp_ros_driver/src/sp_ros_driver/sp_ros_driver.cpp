#include <sp_ros_driver/sp_ros_driver.h>

using namespace servo_platform;

SpRosDriver::SpRosDriver(const char* port_name, ros::NodeHandle& nh, unsigned int ms_sleep) :
_nh(nh),
SpDriver(port_name),
_ms_sleep(ms_sleep)
{
    _current_roll.store(90);
    _current_pitch.store(90);
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
    _state_pub = _nh.advertise<sp_ros_driver::SpState>("sp_state", 1);

    // Create subscriber
    _command_sub = _nh.subscribe("sp_command", 1, &SpRosDriver::commandCb, this);

    spinner_.start();

    return true;
}

void SpRosDriver::commandCb(const sp_ros_driver::SpCommand& sp_command_msg)
{
    // Update current roll and pitch
    _current_roll = sp_command_msg.roll;
    _current_pitch = sp_command_msg.pitch;
}

void SpRosDriver::run()
{
    while (ros::ok())
    {
        // Read roll and pitch
        int roll = _current_roll;
        int pitch = _current_pitch;
        step(roll, pitch, _state, _ms_sleep);

        // Publish state
        sp_ros_driver::SpState sp_state_msg;
        sp_state_msg.roll_actual = _state.state.roll;
        sp_state_msg.pitch_actual = _state.state.pitch;

        sp_state_msg.roll_desired = roll;
        sp_state_msg.pitch_desired = pitch;

        _state_pub.publish(sp_state_msg);
    }
}