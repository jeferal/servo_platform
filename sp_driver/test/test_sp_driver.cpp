#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include <sp_driver/simple_serial/simple_serial.h>


int main(int argc, char** argv)
{
    // Initialize serial port
    SimpleSerial serial_port("/dev/ttyUSB0", 115200);

    while (true)
    {
        // Create sinusoidal signal from 70 to 110 degrees with a period of 5 seconds
        double angle_1 = 90 + 20 * sin(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0 * 2 * M_PI);
        double angle_2 = 90 + 20 * cos(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0 * 2 * M_PI);

        // Convert this angle to a string of size 3 and fill with 0s in the left
        std::string angle_string_1 = std::to_string((int)angle_1);
        std::string angle_string_2 = std::to_string((int)angle_2);
        angle_string_1 = std::string(3 - angle_string_1.length(), '0') + angle_string_1;
        angle_string_2 = std::string(3 - angle_string_2.length(), '0') + angle_string_2;
        angle_string_1 = angle_string_1 + angle_string_2;

        // Add an a at the beginning
        angle_string_1 = "a" + angle_string_1;

        // Print the angle string
        std::cout << "Sending the following string: " << angle_string_1 << std::endl;

        // Send a string to the serial port
        serial_port.writeString(angle_string_1);

        // Read from the serial port until a newline character is found
        std::string result = serial_port.readUntil('\n');
        
        // Print the result
        std::cout << result << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}