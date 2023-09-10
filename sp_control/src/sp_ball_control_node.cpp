#include "sp_control/sp_ball_control_node.h"

using namespace sp_control;


SpBallControl::SpBallControl(ros::NodeHandle& nh) :
nh_(nh),
set_point_x_(640), set_point_y_(360)
{
}

bool SpBallControl::init()
{
    ROS_INFO("Getting parameters from the parameter server");

    // Get the PID parameters from the parameter server
    nh_.param("kp_x", pid_x_config_.kp, 0.0);
    nh_.param("ki_x", pid_x_config_.ki, 0.0);
    nh_.param("kd_x", pid_x_config_.kd, 0.0);
    nh_.param("kp_y", pid_y_config_.kp, 0.0);
    nh_.param("ki_y", pid_y_config_.ki, 0.0);
    nh_.param("kd_y", pid_y_config_.kd, 0.0);

    nh_.getParam("/sp_ros_driver_node/start_roll", start_pitch_);
    nh_.getParam("/sp_ros_driver_node/start_pitch", start_roll_);

    ROS_INFO_STREAM("Start roll: " << start_roll_);
    ROS_INFO_STREAM("Start pitch: " << start_pitch_);

    // Subscribe to the ball position topic
    ball_position_sub_ = nh_.subscribe("state_feedback", 1, &SpBallControl::ballPositionCallback, this);

    // Subscribe to set point
    set_point_sub_ = nh_.subscribe("set_point", 1, &SpBallControl::setPointCallback, this);

    // Publish the servo platform command
    sp_command_pub_ = nh_.advertise<sp_ros_driver::SpCommand>("control_action", 1);

    // Publisher PID x state
    pid_state_pub_ = nh_.advertise<sp_control::BallControlPid>("pid_state", 1);

    // dynamic reconfigure
    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sp_control::SpBallControlConfig>());
    // TODO: Use lambas instead of boost::bind
    function_cb_ = boost::bind(&SpBallControl::dynamicReconfigureCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(function_cb_);

    return true;
}

void SpBallControl::resetState()
{
    pid_x_state_.error = 0;
    pid_x_state_.error_prev = 0;
    pid_x_state_.error_dot = 0;
    pid_x_state_.error_sum = 0;

    pid_y_state_.error = 0;
    pid_y_state_.error_prev = 0;
    pid_y_state_.error_dot = 0;
    pid_y_state_.error_sum = 0;
}

void SpBallControl::getState(std::vector<double>& state)
{
    state.clear();
    state.emplace_back(pid_x_state_.error);
    state.emplace_back(pid_x_state_.error_prev);
    state.emplace_back(pid_x_state_.error_dot);
    state.emplace_back(pid_x_state_.error_sum);

    state.emplace_back(pid_y_state_.error);
    state.emplace_back(pid_y_state_.error_prev);
    state.emplace_back(pid_y_state_.error_dot);
    state.emplace_back(pid_y_state_.error_sum);
}

void SpBallControl::dynamicReconfigureCallback(sp_control::SpBallControlConfig& config, uint32_t level)
{
    // Protect the parameters with a scoped mutex
    std::lock_guard<std::mutex> guard(configuration_mutex_);

    // Update the parameters
    pid_x_config_.kp = config.kp_x;
    pid_x_config_.ki = config.ki_x;
    pid_x_config_.kd = config.kd_x;

    pid_y_config_.kp = config.kp_y;
    pid_y_config_.ki = config.ki_y;
    pid_y_config_.kd = config.kd_y;

    set_point_x_ = config.set_point_x;
    set_point_y_ = config.set_point_y;
}

void SpBallControl::setPointCallback(const sp_ros_driver::SpCommand::ConstPtr& msg)
{
    // Protect the set points with a scoped mutex
    std::lock_guard<std::mutex> guard(configuration_mutex_);

    set_point_x_ = msg->set_point_roll;
    set_point_y_ = msg->set_point_pitch;
}

void SpBallControl::calculate_error_(const double &actual_x, const double &actual_y)
{
    pid_x_state_.error = - (set_point_x_ - actual_x);
    pid_y_state_.error = (set_point_y_ - actual_y);

    pid_x_state_.error_sum += pid_x_state_.error;
    pid_y_state_.error_sum += pid_y_state_.error;

    // Saturate error sum
    saturate_(pid_x_state_.error_sum, -100, 100);
    saturate_(pid_y_state_.error_sum, -100, 100);

    pid_x_state_.error_dot = pid_x_state_.error - pid_x_state_.error_prev;
    pid_y_state_.error_dot = pid_y_state_.error - pid_y_state_.error_prev;
}

void SpBallControl::update_error_()
{
    // Update the internal error x
    pid_x_state_.error_prev = pid_x_state_.error;

    // Update the internal error y
    pid_y_state_.error_prev = pid_y_state_.error;
}

void SpBallControl::step(const double& actual_x, const double& actual_y,
                         double& action_roll, double& action_pitch)
{
    calculate_error_(actual_x, actual_y);

    // Compute the PID for X
    pid_x_state_.action_p = pid_x_config_.kp * pid_x_state_.error;
    pid_x_state_.action_i = pid_x_config_.ki * pid_x_state_.error_sum;
    pid_x_state_.action_d = pid_x_config_.kd * pid_x_state_.error_dot;

    // Compute the PID for Y
    pid_y_state_.action_p = pid_y_config_.kp * pid_y_state_.error;
    pid_y_state_.action_i = pid_y_config_.ki * pid_y_state_.error_sum;
    pid_y_state_.action_d = pid_y_config_.kd * pid_y_state_.error_dot;

    pid_x_state_.action = pid_x_state_.action_p + pid_x_state_.action_i + pid_x_state_.action_d + start_roll_;
    pid_y_state_.action = pid_y_state_.action_p + pid_y_state_.action_i + pid_y_state_.action_d + start_pitch_;

    action_roll = pid_x_state_.action;
    action_pitch = pid_y_state_.action;

    // Saturate roll and pitch
    saturate_(pid_x_state_.action, 60, 130);
    saturate_(pid_y_state_.action, 60, 130);

    // Update internal state
    update_error_();
}

void SpBallControl::ballPositionCallback(const sp_perception::SpTrackingOutput::ConstPtr& msg)
{
    // Protect reading from parameters and set points with a scoped mutex
    std::lock_guard<std::mutex> guard(configuration_mutex_);

    // Step
    double action_roll, action_pitch;
    step(msg->position_x, msg->position_y, action_roll, action_pitch);

    sp_ros_driver::SpCommand sp_command;
    // TODO: Fix convention in the driver
    sp_command.set_point_pitch = action_roll;
    sp_command.set_point_roll = action_pitch;

    // Add the time stamp
    sp_command.header.stamp = ros::Time::now();

    // Create the PID state message for X
    sp_control::BallControlPid pid_state_msg;
    pid_state_msg.set_point_x = set_point_x_;
    pid_state_msg.process_value_x = msg->position_x;
    pid_state_msg.pid_state_x.error = pid_x_state_.error;
    pid_state_msg.pid_state_x.error_dot = pid_x_state_.error_dot;
    pid_state_msg.pid_state_x.p_error = pid_x_state_.error;
    pid_state_msg.pid_state_x.i_error = pid_x_state_.error_sum;
    pid_state_msg.pid_state_x.d_error = pid_x_state_.error_dot;
    pid_state_msg.pid_state_x.p_term = pid_x_state_.action_p;
    pid_state_msg.pid_state_x.i_term = pid_x_state_.action_i;
    pid_state_msg.pid_state_x.d_term = pid_x_state_.action_d;
    pid_state_msg.pid_state_x.i_max = 100;
    pid_state_msg.pid_state_x.i_min = -100;
    pid_state_msg.pid_state_x.output = pid_x_state_.action;

    // Create the PID state message for Y
    pid_state_msg.set_point_y = set_point_y_;
    pid_state_msg.process_value_y = msg->position_y;
    pid_state_msg.pid_state_y.error = pid_y_state_.error;
    pid_state_msg.pid_state_y.error_dot = pid_y_state_.error_dot;
    pid_state_msg.pid_state_y.p_error = pid_y_state_.error;
    pid_state_msg.pid_state_y.i_error = pid_y_state_.error_sum;
    pid_state_msg.pid_state_y.d_error = pid_y_state_.error_dot;
    pid_state_msg.pid_state_y.p_term = pid_y_state_.action_p;
    pid_state_msg.pid_state_y.i_term = pid_y_state_.action_i;
    pid_state_msg.pid_state_y.d_term = pid_y_state_.action_d;
    pid_state_msg.pid_state_y.i_max = 100;
    pid_state_msg.pid_state_y.i_min = -100;
    pid_state_msg.pid_state_y.output = pid_y_state_.action;

    // Update time stamps
    sp_command.header.stamp = ros::Time::now();
    pid_state_msg.pid_state_x.header.stamp = ros::Time::now();
    pid_state_msg.pid_state_y.header.stamp = ros::Time::now();

    // Publish
    sp_command_pub_.publish(sp_command);
    pid_state_pub_.publish(pid_state_msg);
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