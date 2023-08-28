#include <atomic>

#include <ros/ros.h>

#include <sp_driver/sp_driver.h>

#include <sp_ros_driver/SpState.h>
#include <sp_ros_driver/SpCommand.h>

namespace servo_platform
{

    class SpRosDriver : public SpDriver
    {
    public:
        /// @brief 
        SpRosDriver(const char* port_name, ros::NodeHandle& nh, int start_roll = 90, int start_pitch = 90);

        /// @brief 
        /// @return 
        bool init();

        /// @brief 
        void run();

    private:

        /// @brief 
        /// @param sp_command_msg 
        void commandCb(const sp_ros_driver::SpCommand& sp_command_msg);

        // Node handle
        ros::NodeHandle _nh;
        // Publisher of state
        ros::Publisher _state_pub;
        // Subscription for command message
        ros::Subscriber _command_sub;

        // Async spinner for the callback queue
        ros::AsyncSpinner spinner_ = ros::AsyncSpinner(1);

        // Structure State
        SpDataStruct _state;

        std::atomic<int> _current_roll;
        std::atomic<int> _current_pitch;
    };
}
