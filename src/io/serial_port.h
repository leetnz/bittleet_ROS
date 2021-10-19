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
    virtual void shutdown() = 0;

    virtual bool ready() const = 0;
    virtual bool write(const char* bytes, int len) = 0;
};

class SerialPortPigpio : public SerialPort {
public:
    SerialPortPigpio();
    ~SerialPortPigpio();

    void shutdown();

    bool ready() const;
    bool write(const char* bytes, int len);
    
private:
    int _pi = -1;
    int _h = -1;
};

class SerialPortFake : public SerialPort {
public:
    SerialPortFake() = default;

    void shutdown() {}

    bool ready() const { return true; }
    bool write(const char* bytes, int len) { return true; }
};

#endif // _BITTLEET_ROS_SERIAL_PORT_H_