#include <thread>
#include <chrono>
#include <iostream>

#include <sp_driver/sp_driver.h>


using namespace servo_platform;

SpDriver::SpDriver(const char *port_name) :
_port_name(port_name)
{
}

auto SpDriver::init() -> int
{
    _serial_com = std::make_unique<SimpleSerial>(_port_name, 115200);

    return 0;
}

auto SpDriver::step(int roll, int pitch, SpDataStruct &sp_data_struct) -> int
{
    createMessage(roll, pitch);
    _serial_com->writeString(_message);
    // Measure time taken by reading
    auto start = std::chrono::system_clock::now();
    std::string result = _serial_com->readUntil('e');
    auto end = std::chrono::system_clock::now();

    // Print the time
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Read until time: " << elapsed_seconds.count() << "s\n";
    // Get roll and pitch from response
    processResponse(result, sp_data_struct);

    return 0;
}

void SpDriver::processResponse(std::string& message, SpDataStruct& sp_data_struct)
{
    // Get roll angle from response
    unsigned first = message.find("r");
    unsigned last = message.find("p");
    sp_data_struct.state.roll = std::stoi(message.substr(first+1, last-first-1));
    // Get pitch
    first = message.find("p");
    last = message.find("t");
    sp_data_struct.state.pitch = std::stoi(message.substr(first+1, last-first-1));
    // Get time
    first = message.find("t");
    last = message.find("e");
    sp_data_struct.time_com = std::stoi(message.substr(first+1, last-first-1));
}

void SpDriver::createMessage(int roll, int pitch)
{
    _message.clear();
    _message = "a";
    // Size of roll must be 3 digits with 0s in the left if needed
    if (roll < 10)
        _message += "00";
    else if (roll < 100)
        _message += "0";
    _message += std::to_string(roll);
    if (pitch < 10)
        _message += "00";
    else if (pitch < 100)
        _message += "0";
    _message += std::to_string(pitch);
}

auto SpDriver::getMessage() -> std::string
{
    return _message;
}
