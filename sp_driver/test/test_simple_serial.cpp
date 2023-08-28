#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <sp_driver/simple_serial/simple_serial.h>


TEST(testSimpleSerial, testWriteAndReceive)
{
    // Initialize serial port
    std::unique_ptr<SimpleSerial> serial_port = std::make_unique<SimpleSerial>("/dev/ttyUSB0", 115200);

    // Write to roll 90 and pitch 90
    serial_port->writeString("a090090");

    // Read response
    std::string result = serial_port->readUntil('e');

    // Get roll angle from response
    unsigned first = result.find("r");
    unsigned last = result.find("p");
    int roll = std::stoi(result.substr(first+1, last-first-1));
    // Get pitch
    first = result.find("p");
    last = result.find("t");
    int pitch = std::stoi(result.substr(first+1, last-first-1));

    // Check if roll and pitch are equal or close to 90
    EXPECT_NEAR(roll, 90, 2);
    EXPECT_NEAR(pitch, 90, 2);
}

TEST(testSimpleSerial, testWriteAndReceiveStreaming)
{
    // Initialize serial port
    std::unique_ptr<SimpleSerial> serial_port = std::make_unique<SimpleSerial>("/dev/ttyUSB0", 115200);

    // Write to roll 90 and pitch 90
    serial_port->writeString("a090090");
    // Get response
    std::string result = serial_port->readUntil('e');
    // Get roll angle from response
    unsigned first = result.find("r");
    unsigned last = result.find("p");
    int roll = std::stoi(result.substr(first+1, last-first-1));
    // Get pitch
    first = result.find("p");
    last = result.find("t");
    int pitch = std::stoi(result.substr(first+1, last-first-1));

    EXPECT_NEAR(roll, 90, 25);
    EXPECT_NEAR(pitch, 90, 25);

    // Perform the test for 5 seconds
    auto start = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < 5000) // 5 seconds
    {
        // Create sinusoidal signal from 70 to 110 degrees with a period of 5 seconds
        int roll_set_point = 90 + 30 * sin(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 2000.0 * 2 * M_PI);
        int pitch_set_point = 90 + 30 * cos(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 2000.0 * 2 * M_PI);

        // Convert this angle to a string of size 3 and fill with 0s in the left
        std::string roll_set_point_str = std::to_string((int)roll_set_point);
        std::string pitch_set_point_str = std::to_string((int)pitch_set_point);
        roll_set_point_str = std::string(3 - roll_set_point_str.length(), '0') + roll_set_point_str;
        pitch_set_point_str = std::string(3 - pitch_set_point_str.length(), '0') + pitch_set_point_str;
        roll_set_point_str = roll_set_point_str + pitch_set_point_str;

        // Add an a at the beginning
        roll_set_point_str = "a" + roll_set_point_str;

        // Send a string to the serial port
        serial_port->writeString(roll_set_point_str);

        // Read from the serial port until a newline character is found
        std::string result = serial_port->readUntil('e');

        // Get roll angle from response
        unsigned first = result.find("r");
        unsigned last = result.find("p");
        int roll = std::stoi(result.substr(first+1, last-first-1));
        // Get pitch
        first = result.find("p");
        last = result.find("t");
        int pitch = std::stoi(result.substr(first+1, last-first-1));

        EXPECT_NEAR(roll, roll_set_point, 1);
        EXPECT_NEAR(pitch, pitch_set_point, 1);
    }
}
