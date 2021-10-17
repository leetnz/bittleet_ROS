#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pigpiod_if2.h>
#include "io/serial_port.h"
#include "joy/xbox.h"

int main(int argc, char **argv)
{
    // SerialPort serial{};
    // if (serial.ready() == false) {
    //     return 1;
    // }

    XboxController controller{};
    controller.registerButtonPress(
        XboxController::ButtonEvent{
            .button = XboxController::Button::X,
            .callback = []() -> void { 
                ROS_INFO("X Button Pressed");
            }
        }  
    );

    controller.registerAnalog(
        XboxController::AnalogEvent{
            .input = XboxController::Analog::LTRIGGER,
            .callback = [](float value) -> void { 
                ROS_INFO("Trigger: %f", value);
            }
        }  
    );

    ros::init(argc, argv, "xbox_listener");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("joy", 10, &XboxController::callback, &controller);

    
    ros::spin();

    return 0;
}
