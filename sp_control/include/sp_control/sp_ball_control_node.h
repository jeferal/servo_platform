#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <sp_ros_driver/SpCommand.h>
#include <sp_perception/SpTrackingOutput.h>
#include <sp_control/SpBallControlConfig.h>

#include <sp_control/BallControlPid.h>

namespace sp_control
{
    class SpBallControl
    {
        public:

            // Structure for PID config
            struct PIDConfig
            {
                double kp = 0;
                double ki = 0;
                double kd = 0;
            };

            // Structure for PID state
            struct PIDState
            {
                // Errors
                double error = 0;
                double error_prev = 0;
                double error_dot = 0;
                double error_sum = 0;
                // Actions
                double action_p = 0;
                double action_i = 0;
                double action_d = 0;
                double action = 0;
            };

            /// @brief 
            /// @param nh 
            SpBallControl(ros::NodeHandle& nh);

            bool init();

            /// @brief 
            /// @param msg 
            void ballPositionCallback(const sp_perception::SpTrackingOutput::ConstPtr& msg);

            /// @brief 
            /// @param msg 
            void setPointCallback(const sp_ros_driver::SpCommand::ConstPtr& msg);

            /// @brief 
            void run();

            // dynamic reconfigure callback
            void dynamicReconfigureCallback(sp_control::SpBallControlConfig& config, uint32_t level);

            void resetState();

            void getState(std::vector<double>& state);

            /// @brief 
            /// @param actual_x
            /// @param actual_y 
            /// @param error_x
            /// @param error_y 
            /// @param action_roll 
            /// @param action_pitch 
            void step(const double& actual_x, const double& actual_y,
                      double& action_roll, double& action_pitch);

        private:

            void saturate_(double& value, double min, double max);

            void update_error_();

            /// @brief 
            /// @param actual_x 
            /// @param actual_y 
            /// @param error_x 
            /// @param error_y 
            void calculate_error_(const double &actual_x, const double &actual_y);

            ros::NodeHandle nh_;

            // Subscriber for the ball position
            ros::Subscriber ball_position_sub_;

            // Publisher for the servo platform command
            ros::Publisher sp_command_pub_;

            // Publisher of PID x state
            ros::Publisher pid_x_state_pub_;
            // Publisher of PID y state
            ros::Publisher pid_y_state_pub_;

            // Subscriber for the set point
            ros::Subscriber set_point_sub_;

            // PID parameters
            PIDConfig pid_x_config_, pid_y_config_;
            
            // Internal variables of PID
            PIDState pid_x_state_, pid_y_state_;

            // Current set point
            double set_point_x_;
            double set_point_y_;

            // Current roll and pitch
            double start_roll_;
            double start_pitch_;

            // dynamic reconfigure
            std::unique_ptr<dynamic_reconfigure::Server<sp_control::SpBallControlConfig>>
                dynamic_reconfigure_server_;
            dynamic_reconfigure::Server<sp_control::SpBallControlConfig>::CallbackType function_cb_;

            std::mutex configuration_mutex_;
    };

}   // namespace sp_control
