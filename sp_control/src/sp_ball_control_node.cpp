#include "sp_control/sp_ball_control_node.h"

using namespace sp_control;


SpBallControl::SpBallControl(ros::NodeHandle& nh) :
nh_(nh),
error_x_(0), error_y_(0),
error_sum_x_(0), error_sum_y_(0),
set_point_x(0), set_point_y(0)
{
}

bool SpBallControl::init()
{
    // Get the PID parameters from the parameter server
    nh_.param("kp", kp_, 0.0);
    nh_.param("ki", ki_, 0.0);
    nh_.param("kd", kd_, 0.0);

    ROS_INFO("Getting parameters from the parameter server");
    nh_.getParam("/sp_ros_driver_node/start_roll", start_pitch_);
    nh_.getParam("/sp_ros_driver_node/start_pitch", start_roll_);
    ROS_INFO_STREAM("Start roll: " << start_roll_);
    ROS_INFO_STREAM("Start pitch: " << start_pitch_);

    // Subscribe to the ball position topic
    ball_position_sub_ = nh_.subscribe("/sp_ball_position", 1, &SpBallControl::ballPositionCallback, this);

    // Publish the servo platform command
    sp_command_pub_ = nh_.advertise<sp_ros_driver::SpCommand>("/sp_ros_driver_node/sp_command", 1);

    set_point_x = 300;
    set_point_y = 250;

    return true;
}

void SpBallControl::ballPositionCallback(const sp_ros_driver::SpCommand::ConstPtr& msg)
{
    ROS_INFO("Ball position callback");
    double error_x = - (set_point_x - msg->roll);
    double error_y = (set_point_y - msg->pitch);

    // Compute the PID for X
    double p_x = kp_ * error_x;
    double i_x = ki_ * error_sum_x_;
    double d_x = kd_ * (error_x - error_x_);

    // Update the error sum
    error_sum_x_ += error_x;
    // Update the error
    error_x_ = error_x;

    // Compute the PID for Y
    double p_y = kp_ * error_y;
    double i_y = ki_ * error_sum_y_;
    double d_y = kd_ * (error_y - error_y_);

    // Update the error sum
    error_sum_y_ += error_y;
    // Update the error
    error_y_ = error_y;

    double roll_action = p_x + i_x + d_x + start_roll_;
    double pitch_action = p_y + i_y + d_y + start_pitch_;

    // Saturate roll and pitch
    saturate_(roll_action, 60, 130);
    saturate_(pitch_action, 60, 130);

    sp_ros_driver::SpCommand sp_command;
    sp_command.pitch = roll_action;
    sp_command.roll = pitch_action;

    // Print roll and pitch
    ROS_INFO("Roll: %f, Pitch: %f", sp_command.roll, sp_command.pitch);

    sp_command_pub_.publish(sp_command);
}

void SpBallControl::saturate_(double& value, double min, double max)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }
}

void SpBallControl::run()
{
    ros::spin();
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "sp_ball_control_node");

    ros::NodeHandle nh("~");

    SpBallControl sp_ball_control(nh);

    sp_ball_control.init();

    sp_ball_control.run();

    return 0;
}