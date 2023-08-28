#include <memory>

#include "simple_serial/simple_serial.h"

namespace servo_platform
{
    struct State
    {
        int roll;
        int pitch;
    };

    struct SpDataStruct
    {
        State state;
        int time_com;
    };

    class SpDriver
    {

        public:

            /// @brief 
            /// @param port_name 
            explicit SpDriver(const char* port_name);

            /// @brief 
            /// @return 
            auto init() -> int;

            /// @brief 
            /// @param roll 
            /// @param pitch 
            /// @param sp_data_struct 
            /// @return 
            // TODO: Optimize communication to remove sleep
            auto step(int roll, int pitch, SpDataStruct &sp_data_struct, unsigned int ms_sleep = 50) -> int;

            /// @brief 
            /// @return 
            auto getMessage() -> std::string;

            /// @brief 
            /// @param roll 
            /// @param pitch 
            void createMessage(int roll, int pitch);

            /// @brief 
            /// @param message 
            /// @param sp_data_struct 
            void processResponse(std::string& message, SpDataStruct& sp_data_struct);

        private:

            std::unique_ptr<SimpleSerial> _serial_com;
            std::string _message;
            const char* _port_name;
    };

}   // namespace servo_platform