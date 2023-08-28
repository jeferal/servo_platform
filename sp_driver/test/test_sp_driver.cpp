#include <thread>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>

#include <sp_driver/sp_driver.h>


TEST(testSpDriver, testCreateMessage)
{
    servo_platform::SpDriver sp_driver("/dev/ttyUSB0");
    sp_driver.createMessage(90, 90);
    std::string message = sp_driver.getMessage();
    EXPECT_EQ(message, "a090090");

    message.clear();
    sp_driver.createMessage(179, 002);
    message = sp_driver.getMessage();
    EXPECT_EQ(message, "a179002");
}

TEST(testSpDriver, testProcessResponse)
{
    servo_platform::SpDriver sp_driver("/dev/ttyUSB0");
    std::string message = "r090p090t000e";
    servo_platform::SpDataStruct sp_data_struct;
    sp_driver.processResponse(message, sp_data_struct);
    EXPECT_EQ(sp_data_struct.state.roll, 90);
    EXPECT_EQ(sp_data_struct.state.pitch, 90);
    EXPECT_EQ(sp_data_struct.time_com, 0);

    message = "r005p150t010e";
    sp_data_struct;
    sp_driver.processResponse(message, sp_data_struct);
    EXPECT_EQ(sp_data_struct.state.roll, 5);
    EXPECT_EQ(sp_data_struct.state.pitch, 150);
    EXPECT_EQ(sp_data_struct.time_com, 10);
}

TEST(testSpDriver, testCommunication)
{

    servo_platform::SpDriver sp_driver("/dev/ttyUSB0");
    sp_driver.init();
    servo_platform::SpDataStruct sp_data_struct;

    // Perform the test for 5 seconds
    auto start = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < 5000) // 5 seconds
    {
        // Create sinusoidal signal from 70 to 110 degrees with a period of 5 seconds
        int roll_set_point = 90 + 30 * sin(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 2000.0 * 2 * M_PI);
        int pitch_set_point = 90 + 30 * cos(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 2000.0 * 2 * M_PI);

        sp_driver.step(roll_set_point, pitch_set_point, sp_data_struct);

        EXPECT_NEAR(sp_data_struct.state.roll, roll_set_point, 2);
        EXPECT_NEAR(sp_data_struct.state.pitch, pitch_set_point, 2);
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}