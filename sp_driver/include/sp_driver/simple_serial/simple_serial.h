#include <string>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>

#include <termios.h> // For serial port settings
#include <unistd.h>  // For sleep function


class SimpleSerial
{

public:

    // Constructor
    SimpleSerial(const char* port_name, unsigned int baud_rate);

    /// @brief 
    /// @param message 
    /// @return 
    auto writeString(const std::string& message) -> void;

    /// @brief 
    /// @param character 
    /// @return 
    auto readUntil(char character) -> std::string;

private:

    /// @brief 
    /// @param baud_rate 
    /// @return 
    auto configurePort(unsigned int baud_rate) -> int;

    int _fd;

};