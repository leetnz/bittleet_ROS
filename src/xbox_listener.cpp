#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pigpiod_if2.h>
#include "io/serial_port.h"
#include "joy/xbox.h"

static SerialPort serial;
static XboxController controller;

void at_signal(int i) {
   serial.shutdown();
   exit(0);
}

int main(int argc, char **argv)
{
    if (serial.ready() == false) {
        return 1;
    }

    using ButtonMap = std::map<XboxController::Button, std::string>;
    ButtonMap buttonMap = {
        {XboxController::Button::X, "kh"},
        {XboxController::Button::Y, "ks"},
        {XboxController::Button::A, "ke"},
        {XboxController::Button::B, "kT"},
    };
    
    for (ButtonMap::const_reference& pair : buttonMap) {
        controller.registerButtonPress(
            XboxController::ButtonEvent{
                .button = pair.first,
                .callback = [pair]() -> void {
                    ROS_INFO("Writing %s", pair.second.c_str());
                    serial.write(pair.second.c_str(), pair.second.size());
                }
            }  
        );
    }

    // controller.registerAnalog(
    //     XboxController::AnalogEvent{
    //         .input = XboxController::Analog::LTRIGGER,
    //         .callback = [](float value) -> void { 
    //             ROS_INFO("Trigger: %f", value);
    //         }
    //     }  
    // );

    ros::init(argc, argv, "xbox_listener");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("joy", 10, &XboxController::callback, &controller);

    ros::spin();

    return 0;
}
