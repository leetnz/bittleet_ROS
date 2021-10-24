#include <functional>
#include <ros/ros.h>
#include <csignal>
#include <sstream>

#include "io/serial_port.h"
#include "joy/xbox.h"
#include "stream/stream.h"
#include "bittleet/Stream.h"

static SerialPort* serial;
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

struct Config {
    std::string addr;
    bool debug;
    bool h264Cam;
};

bool processArguments(ros::NodeHandle& handle, Config& config) {
    config = Config{
        .addr = "",
        .debug = false,
        .h264Cam = true,
    };

    if (!handle.getParam("addr", config.addr)) {
        ROS_ERROR("Requires param _addr. \n"
            "e.g. `rosrun bittleet xbox_listener _addr:=192.168.0.7`");
        return false;
    }

    ROS_INFO("Set address to %s", config.addr.c_str());

    handle.getParam("debug", config.debug);
    handle.getParam("h264", config.h264Cam);
    return true;
}

std::string constructStream(const Config& config){
    std::stringstream ss;
    if (config.h264Cam) {
        ss << "uvch264src iframe-period=1000 device=/dev/video0";
    } else {
        ss << "v4l2src device=/dev/video0";
    }
    ss << " ! video/x-h264,width=640,height=480,framerate=30/1,profile=baseline";
    ss << " ! h264parse";
    ss << " ! rtph264pay config-interval=-1 pt=96";
    ss << " ! gdppay";
    ss << " ! tcpserversink host=0.0.0.0 port=5000";

    return ss.str();
}

void atSignal(int i) {
    if (serial != NULL) {
        serial->shutdown();
    }
    if (stream != NULL) {
        stream->shutdown();
    }
    exit(0);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, atSignal);

    ros::init(argc, argv, "xbox_listener");
    ros::NodeHandle localHandle("~");
    ros::NodeHandle globalHandle;

    Config config;
    if (processArguments(localHandle, config) == false) {
        return 1;
    }

    if (config.debug) {
        serial = new SerialPortFake();
    } else {
        serial = new SerialPortPigpio();
    }

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
            .callback = [&config]() -> void {
                if (stream == NULL) {
                    ROS_INFO("Starting Video Stream");
                    stream = new Stream(constructStream(config));
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

    


    ros::Subscriber sub = globalHandle.subscribe("joy", 10, &XboxController::callback, &controller);

    ros::Publisher active_pub = globalHandle.advertise<bittleet::Stream>("stream", 1);

    ros::Rate loop_rate(10);

    bittleet::Stream msg;
    msg.addr = config.addr;
    msg.port = "5000";

    while (ros::ok())
    {
        msg.active = (stream != NULL);

        active_pub.publish(msg);

        if (stream != NULL) {
            if (stream->isRunning() == false) {
                ROS_WARN("Unexpected end of stream");
                stream->shutdown();
                delete stream;
                stream = NULL;
            }
        }

        ros::spinOnce(); // Wait for any callbacks
        loop_rate.sleep();
    }

    return 0;
}
