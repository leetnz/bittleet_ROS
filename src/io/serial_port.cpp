//
// Serial Port
//
// Hoani Bryson (github.com/hoani)
// Copyright (c) 2021 Leetware Limited.
// License - MIT
//

#include "ros/ros.h"
#include "serial_port.h"
#include <pigpiod_if2.h>

SerialPortPigpio::SerialPortPigpio() {
    _pi = pigpio_start("localhost", "8888");
    if (_pi < 0) {
        ROS_ERROR("Connecting to pigpiod failed");
        return;
    }

    char serialPort[11] = "/dev/ttyS0";
    _h = serial_open(_pi, serialPort, 115200, 0);
    if (_h < 0) {
        ROS_ERROR("Serial port unavaliable");
        return;
    }
}

SerialPortPigpio::~SerialPortPigpio() {
    shutdown();
}

void SerialPortPigpio::shutdown() {
    if (_h >= 0) {
        serial_close(_pi, _h);
    }
    if (_pi >= 0) {
        pigpio_stop(_pi);
    }
    _h = -1;
    _pi = -1;
}

bool SerialPortPigpio::ready() const {
    return ((_h >= 0) && (_pi >= 0));
}

bool SerialPortPigpio::write(const char* bytes, int len){
    char toWrite[len];
    std::memcpy(toWrite, bytes, len);
    int result  = serial_write(_pi, _h, toWrite, (unsigned)len);
    return (result == 0);
}
