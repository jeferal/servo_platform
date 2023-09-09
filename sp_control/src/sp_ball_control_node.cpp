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
    ROS_INFO("Getting parameters from the parameter server");

    // Get the PID parameters from the parameter server
    nh_.param("kp_x", kp_x_, 0.0);
    nh_.param("ki_x", ki_x_, 0.0);
    nh_.param("kd_x", kd_x_, 0.0);
    nh_.param("kp_y", kp_y_, 0.0);
    nh_.param("ki_y", ki_y_, 0.0);
    nh_.param("kd_y", kd_y_, 0.0);

    nh_.getParam("/sp_ros_driver_node/start_roll", start_pitch_);
    nh_.getParam("/sp_ros_driver_node/start_pitch", start_roll_);
    ROS_INFO_STREAM("Start roll: " << start_roll_);
    ROS_INFO_STREAM("Start pitch: " << start_pitch_);

    // Subscribe to the ball position topic
    ball_position_sub_ = nh_.subscribe("state_feedback", 1, &SpBallControl::ballPositionCallback, this);

    // Publish the servo platform command
    sp_command_pub_ = nh_.advertise<sp_ros_driver::SpCommand>("control_action", 1);

    nh_.getParam("/sp_ros_driver_node/set_point_x", set_point_x);
    nh_.getParam("/sp_ros_driver_node/set_point_y", set_point_y);

    // dynamic reconfigure
    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sp_control::SpBallControlConfig>());
    // TODO: Use lambas instead of boost::bind
    function_cb_ = boost::bind(&SpBallControl::dynamicReconfigureCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(function_cb_);

    return true;
}

void SpBallControl::dynamicReconfigureCallback(sp_control::SpBallControlConfig& config, uint32_t level)
{
    // Update the parameters
    // TODO: Make thread safe
    kp_x_ = config.kp_x;
    kd_x_ = config.kd_x;
    ki_x_ = config.ki_x;

    kp_y_ = config.kp_y;
    kd_y_ = config.kd_y;
    ki_y_ = config.ki_y;

    set_point_x = config.set_point_x;
    set_point_y = config.set_point_y;
}

void SpBallControl::calculate_error_(const double &actual_x, const double &actual_y, double& error_x, double& error_y)
{
    error_x = - (set_point_x - actual_x);
    error_y = (set_point_y - actual_y);

    // Update the error sum x
    error_sum_x_ += error_x;
    // Saturate error sum
    saturate_(error_sum_x_, -100, 100);
    // Update the internal error x
    error_x_ = error_x;

    // Update the error sum y
    error_sum_y_ += error_y;
    saturate_(error_sum_y_, -100, 100);
    // Update the internal error y
    error_y_ = error_y;
}

void SpBallControl::ballPositionCallback(const sp_perception::SpTrackingOutput::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5, "Ball position callback");

    // Calculate the error
    double error_x, error_y;
    calculate_error_(msg->position_x, msg->position_y, error_x, error_y);

    // Compute the PID for X
    double p_x = kp_x_ * error_x;
    double i_x = ki_x_ * error_sum_x_;
    double d_x = kd_x_ * (error_x - error_x_);    // TODO: Add deviding by time

    // Compute the PID for Y
    double p_y = kp_y_ * error_y;
    double i_y = ki_y_ * error_sum_y_;
    double d_y = kd_y_ * (error_y - error_y_);

    double roll_action = p_x + i_x + d_x + start_roll_;
    double pitch_action = p_y + i_y + d_y + start_pitch_;

    // Saturate roll and pitch
    saturate_(roll_action, 60, 130);
    saturate_(pitch_action, 60, 130);

    sp_ros_driver::SpCommand sp_command;
    sp_command.set_point_pitch = roll_action;
    sp_command.set_point_roll = pitch_action;

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