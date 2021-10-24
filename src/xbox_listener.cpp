#include <functional>
#include <ros/ros.h>
#include "io/serial_port.h"
#include "joy/xbox.h"
#include "stream/stream.h"
#include "std_msgs/Bool.h"

static SerialPort* serial = new SerialPortPigpio();
static XboxController controller;
static Stream* stream;

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
    if (stream != NULL) {
        stream->shutdown();
    }
    exit(0);
}

int main(int argc, char **argv)
{
    if (serial->ready() == false) {
        ROS_ERROR("Failed to open serial port");
        return 1;
    }

    Stream::mainInit(argc, argv);


    using ButtonMap = std::map<XboxController::Button, std::string>;
    ButtonMap buttonMap = {
        {XboxController::Button::X, "kh"},
        {XboxController::Button::Y, "ks"},
        {XboxController::Button::A, "ke"},
        {XboxController::Button::B, "kT"},
        {XboxController::Button::LTOGGLE, "kb"},
        {XboxController::Button::RB, "g"}, // enable/disable gyro.
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

    controller.registerButtonPress(
        XboxController::ButtonEvent{
            .button = XboxController::Button::LB,
            .callback = []() -> void {
                if (stream == NULL) {
                    ROS_INFO("Starting Video Stream");
                    // stream = new Stream("videotestsrc pattern=ball ! video/x-raw,width=640,height=480 ! videoconvert ! autovideosink");
                    // stream = new Stream("gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! autovideosink");
                    stream = new Stream("gst-launch-1.0 v4l2src device=/dev/video0 ! "
                        "video/x-h264,width=640,height=480,framerate=30/1 ! "
                        "h264parse ! "
                        "rtph264pay config-interval=1 pt=96 ! "
                        "gdppay ! "
                        "tcpserversink host=127.0.0.1 port=5000");
                } else {
                    ROS_INFO("Stopping Video Stream");
                    stream->shutdown();
                    delete stream;
                    stream = NULL;
                }
            }
        }  
    );

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

    ros::Publisher active_pub = nh.advertise<std_msgs::Bool>("stream_active", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::Bool msg;

        msg.data = (stream != NULL);

        active_pub.publish(msg);

        ros::spinOnce(); // Wait for any callbacks
        loop_rate.sleep();
    }

    return 0;
}
