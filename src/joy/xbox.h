//
// XBox Joy Controller
//
// Hoani Bryson (github.com/hoani)
// Copyright (c) 2021 Leetware Limited.
// License - MIT
//

#ifndef _BITTLEET_ROS_JOY_XBOX_H_
#define _BITTLEET_ROS_JOY_XBOX_H_

#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


class XboxController {
public:
    
    XboxController();

    void callback(const sensor_msgs::Joy::ConstPtr& msg);

    struct ButtonEvent;

    void registerButtonPress(ButtonEvent event);
    void registerButtonRelease(ButtonEvent event);

    enum class Button : int {
        A = 0, B, X, Y, 
        LB, RB, 
        BACK, START, 
        XBOX, 
        LTOGGLE, RTOGGLE, 
        DLEFT, DRIGHT, DUP, DDOWN
    };

    struct ButtonEvent {
        Button button;
        std::function<void()> callback;
    };
    

protected:
    sensor_msgs::Joy _defaultInput();

    bool _checkPressed(Button button, const sensor_msgs::Joy::ConstPtr& current) const;
    bool _checkReleased(Button button, const sensor_msgs::Joy::ConstPtr& current) const;
    bool _checkChanged(Button button, const sensor_msgs::Joy::ConstPtr& current) const;

    sensor_msgs::Joy _last;

    std::vector<ButtonEvent> _buttonPressEvents;
    std::vector<ButtonEvent> _buttonReleaseEvents;

    static constexpr size_t NUM_AXES = 8;
    static constexpr size_t NUM_BUTTONS = 15;
};

#endif // _BITTLEET_ROS_JOY_XBOX_H_
