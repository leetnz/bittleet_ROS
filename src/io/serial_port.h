//
// Serial Port
//
// Hoani Bryson (github.com/hoani)
// Copyright (c) 2021 Leetware Limited.
// License - MIT
//

#ifndef _BITTLEET_ROS_SERIAL_PORT_H_
#define _BITTLEET_ROS_SERIAL_PORT_H_

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    void shutdown();

    bool ready() const;
    bool write(char* bytes, int len);
    
private:
    int _pi = -1;
    int _h = -1;
};

#endif // _BITTLEET_ROS_SERIAL_PORT_H_