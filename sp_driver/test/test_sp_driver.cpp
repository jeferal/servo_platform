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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Write to roll 90 and pitch 90
    serial_port->writeString("a090090");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        double angle_1 = 90 + 20 * sin(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 2000.0 * 2 * M_PI);
        double angle_2 = 90 + 20 * cos(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 2000.0 * 2 * M_PI);

        // Convert this angle to a string of size 3 and fill with 0s in the left
        std::string angle_string_1 = std::to_string((int)angle_1);
        std::string angle_string_2 = std::to_string((int)angle_2);
        angle_string_1 = std::string(3 - angle_string_1.length(), '0') + angle_string_1;
        angle_string_2 = std::string(3 - angle_string_2.length(), '0') + angle_string_2;
        angle_string_1 = angle_string_1 + angle_string_2;

        // Add an a at the beginning
        angle_string_1 = "a" + angle_string_1;

        // Send a string to the serial port
        serial_port->writeString(angle_string_1);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

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

        EXPECT_NEAR(roll, 90, 25);
        EXPECT_NEAR(pitch, 90, 25);
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
