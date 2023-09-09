// Include boost scoped_ptr
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <sp_ros_driver/SpCommand.h>
#include <sp_perception/SpTrackingOutput.h>
#include <sp_control/SpBallControlConfig.h>


namespace sp_control
{
    class SpBallControl
    {

        public:

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

        private:

            void saturate_(double& value, double min, double max);

            void update_error_(const double &error_x, const double &error_y);

            /// @brief 
            /// @param actual_x 
            /// @param actual_y 
            /// @param error_x 
            /// @param error_y 
            void calculate_error_(const double &actual_x, const double &actual_y, double& error_x, double& error_y);

            ros::NodeHandle nh_;

            // Subscriber for the ball position
            ros::Subscriber ball_position_sub_;

            // Publisher for the servo platform command
            ros::Publisher sp_command_pub_;

            // Subscriber for the set point
            ros::Subscriber set_point_sub_;

            // PID parameters
            double kp_x_, kp_y_;
            double ki_x_, ki_y_;
            double kd_x_, kd_y_;

            // Internal variables of PID
            double error_x_, error_y_;
            double error_sum_x_, error_sum_y_;

            // Current set point
            double set_point_x_;
            double set_point_y_;

            // Current roll and pitch
            double start_roll_;
            double start_pitch_;

            // dynamic reconfigure
            boost::scoped_ptr<dynamic_reconfigure::Server<sp_control::SpBallControlConfig>>
                dynamic_reconfigure_server_;
            dynamic_reconfigure::Server<sp_control::SpBallControlConfig>::CallbackType function_cb_;
    };

}   // namespace sp_control
