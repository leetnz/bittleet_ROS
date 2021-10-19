#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pigpiod_if2.h>
#include "io/serial_port.h"
#include "joy/xbox.h"

static SerialPort* serial = new SerialPortPigpio();
static XboxController controller;

class MoveStateMachine {
    public:
        MoveStateMachine() = default;

        void xCallback(float value) {
            xValue = value;
            QuadrantState current = _quadrant();
            if (current != lastQuadrant) {
                lastQuadrant = current;
                {
                    std::string cmd = _quadrantMap[current];
                    ROS_INFO("Writing %s", cmd.c_str());
                    serial->write(cmd.c_str(), cmd.size());
                }
            }
        }

        void yCallback(float value) {
            yValue = value;

            QuadrantState current = _quadrant();
            if (current != lastQuadrant) {
                lastQuadrant = current;
                {
                    std::string cmd = _quadrantMap[current];
                    ROS_INFO("Writing %s", cmd.c_str());
                    serial->write(cmd.c_str(), cmd.size());
                }
            }
        }

        static constexpr size_t NumSpeedStates = 5;
        enum class QuadrantState : int {
            None = 0,
            Forward,
            Left,
            Backward,
            Right,
        };
        static constexpr size_t NumQuadrantStates = 5;


        float xValue = 0.0;
        float yValue = 0.0;
        QuadrantState lastQuadrant = QuadrantState::None;

    private:
        using QuadrantMap = std::map<QuadrantState, std::string>;
        QuadrantMap _quadrantMap = {
            {QuadrantState::None,       "kb"},
            {QuadrantState::Forward,    "kF"},
            {QuadrantState::Left,       "kL"},
            {QuadrantState::Right,      "kR"},
            {QuadrantState::Backward,   "kB"},
        };
        

        QuadrantState _quadrant() {
            const float norm = sqrt(xValue * xValue + yValue * yValue);

            if (norm < 0.2) {
                return QuadrantState::None;
            }

            const float theta = atan2(yValue, xValue);

            if (fabs(theta) < M_PI_4) {
                return QuadrantState::Forward;
            }

            if (fabs(theta) >  3 * M_PI_4) {
                return QuadrantState::Backward;
            }

            if (theta > 0) {
                return QuadrantState::Left;
            }
            
            return QuadrantState::Right;
        }
};

void at_signal(int i) {
   serial->shutdown();
   exit(0);
}

int main(int argc, char **argv)
{
    if (serial->ready() == false) {
        return 1;
    }

    using ButtonMap = std::map<XboxController::Button, std::string>;
    ButtonMap buttonMap = {
        {XboxController::Button::X, "kh"},
        {XboxController::Button::Y, "ks"},
        {XboxController::Button::A, "ke"},
        {XboxController::Button::B, "kT"},
        {XboxController::Button::LTOGGLE, "kb"},
    };
    
    for (ButtonMap::const_reference& pair : buttonMap) {
        controller.registerButtonPress(
            XboxController::ButtonEvent{
                .button = pair.first,
                .callback = [pair]() -> void {
                    ROS_INFO("Writing %s", pair.second.c_str());
                    serial->write(pair.second.c_str(), pair.second.size());
                }
            }  
        );
    }

    MoveStateMachine move{};
    
    
    controller.registerAnalog(
        XboxController::AnalogEvent{
            .input = XboxController::Analog::LTOGGLE_UD,
            .callback = [&move](float value) -> void {
                move.xCallback(value);
            }
        }  
    );

    controller.registerAnalog(
        XboxController::AnalogEvent{
            .input = XboxController::Analog::LTOGGLE_LR,
            .callback = [&move](float value) -> void {
                move.yCallback(value);
            }
        }  
    );

    ros::init(argc, argv, "xbox_listener");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("joy", 10, &XboxController::callback, &controller);

    ros::spin();

    return 0;
}
