
#include <iostream>

#include <sp_driver/simple_serial/simple_serial.h>


SimpleSerial::SimpleSerial(const char* port_name, unsigned int baud_rate)
{

    std::cout << "Opening serial port " << port_name << " at " << baud_rate << " baud" << std::endl;

    // Open the serial port
    _fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    std::cout << "fd: " << _fd << std::endl;

    if (_fd == -1)
    {
        throw std::runtime_error("serialport_init: Unable to open port ");
    }

    // Configure the port
    if (configurePort(baud_rate) != 0)
    {
        throw std::runtime_error("Failed to configure serial port");
    }

    std::cout << "Opened serial port " << port_name << " at " << baud_rate << " baud" << std::endl;
}

SimpleSerial::~SimpleSerial()
{
    // Close the serial port
    std::cout << "Closing serial port" << std::endl;
    close(_fd);
}

auto SimpleSerial::configurePort(unsigned int baud_rate) -> int
{
    // Get current serial port settings
    struct termios serialConfig;

    tcgetattr(_fd, &serialConfig);

    cfmakeraw(&serialConfig);
    serialConfig.c_cflag |= (CLOCAL | CREAD);
    serialConfig.c_iflag &= ~(IXOFF | IXANY);

    // set vtime, vmin, baud rate...
    serialConfig.c_cc[VMIN] = 0;  // you likely don't want to change this
    serialConfig.c_cc[VTIME] = 0; // or this

    cfsetispeed(&serialConfig, B115200);
    cfsetospeed(&serialConfig, B115200);

    // Apply the new settings
    tcsetattr(_fd, TCSANOW, &serialConfig);

    return 0;
}

auto SimpleSerial::writeString(const std::string &message) -> int
{
    // Write the string to the serial port
    return write(_fd, message.c_str(), message.length());
}

auto SimpleSerial::readUntil(char character) -> std::string
{
    // Read the serial port until the desired character is found
    std::string result;
    char buffer[1];
    int n = 0;
    do
    {
        n = read(_fd, buffer, 1);
        result += buffer[0];
    }
    while (buffer[0] != character && n > 0);

    std::cout << "result: " << result << std::endl;
    return result;
}
