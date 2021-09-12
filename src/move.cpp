#include <iostream>
#include <sstream>
#include <csignal>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bittleet/MoveServo.h"

#include <pigpiod_if2.h>

class Move{
public:
    Move(int pi, int handle) : 
            _srv(_n.advertiseService("bittleet_move/move", &Move::command_callback, this)),
            _pi(pi),
            _h(handle){}

    void run() {
        ros::spin();
    }

    bool command_callback(bittleet::MoveServo::Request &req, bittleet::MoveServo::Response &res) {
        if (req.index != 0 && (req.index < 8 || req.index > 15)) {
            ROS_WARN("Index %d invalid", (int)req.index);
            return false;
        }
        if (req.angle < -90.0 || req.angle > 90.0) {
            ROS_WARN("Angle %f out of range", req.angle);
            return false;
        }

        char command[30];
        int n = 0;
        n = sprintf(command, "m %d %.0f ", req.index, req.angle);
        ROS_INFO("Sending command '%s'", command);

        if (n > 0) {
            _write_to_serial(command, n);
        } else {
            return false;
        }
        time_sleep(0.1);
        char response[128] = {0};
        int len = _serial_read(response, 128);

        res.result=std::string(response);

        return true;

    }

private:
    void _write_to_serial(char* bytes, int len){
        int result  = serial_write(_pi, _h, bytes, (unsigned)len);
        if (result != 0) {
            ROS_ERROR("Serial write failed: %d ", result);
        }
    }

    int _serial_read(char *bytes, int max_len) {
        int index = 0;
        int byte = serial_read_byte(_pi, _h);

        while ((byte >= 0) && (index < max_len)) {
            bytes[index] = (char)byte;
            index++;       
            byte = serial_read_byte(_pi, _h);
        }
        return index;
    }

    ros::NodeHandle _n;
    ros::ServiceServer _srv;

    int _pi = 0;
    int _h = 0;
};

static int pi = -1;
static int handle = -1;

void shutdown() {
    std::cout << "Shutting down connection to pigpiod\n";
     if (handle >= 0) {
        serial_close(pi, handle);
    }
    if (pi >= 0) {
        pigpio_stop(pi);
    }
}

void at_signal(int i) {
   shutdown();
   exit(0);
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, at_signal);

    pi = pigpio_start("localhost", "8888");
    if (pi < 0) {
        std::cout << "Connecting to pigpiod failed\n";
        return 1;
    }

    char serialPort[11] = "/dev/ttyS0";
    handle = serial_open(pi, serialPort, 115200, 0);
    if (handle < 0) {
        std::cout << "Serial port unavaliable\n";
        shutdown();
        return 1;
    }

    ros::init(argc, argv, "bittleet_move");

    Move node(pi, handle);
    node.run();

    shutdown();
    return 0;
}