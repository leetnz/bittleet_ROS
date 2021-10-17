//
// XBox Joy Controller
//
// Hoani Bryson (github.com/hoani)
// Copyright (c) 2021 Leetware Limited.
// License - MIT
//

#include "xbox.h"

    
XboxController::XboxController() {
    _last = _defaultInput();
}

void XboxController::callback(const sensor_msgs::Joy::ConstPtr& msg) {
    if ((msg->axes.size() != NUM_AXES) || (msg->buttons.size() != NUM_BUTTONS)) {
        ROS_ERROR("Joy message has the wrong number of inputs for an xbox controller! Please check joy node config.");
        return;
    }

    for (auto event: _buttonPressEvents) {
        if (_checkPressed(event.button, msg)) {
            event.callback();
        }
    }

    for (auto event: _buttonReleaseEvents) {
        if (_checkReleased(event.button, msg)) {
            event.callback();
        }
    }

    for (auto event: _analogEvents) {
        if (_checkChanged(event.input, msg)) {
            const int index = static_cast<int>(event.input);
            event.callback(msg->axes[index]);
        }
    }

    _last = *msg;
}

void XboxController::registerButtonPress(ButtonEvent event) {
    _buttonPressEvents.push_back(event);
}

void XboxController::registerButtonRelease(ButtonEvent event) {
    _buttonReleaseEvents.push_back(event);
}

void XboxController::registerAnalog(AnalogEvent event) {
    _analogEvents.push_back(event);
}


sensor_msgs::Joy XboxController::_defaultInput() {
    sensor_msgs::Joy input;
    input.axes = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    input.buttons = std::vector<int>(15, 0);
    return input;
}

bool XboxController::_checkPressed(Button button, const sensor_msgs::Joy::ConstPtr& current) const {
    const int index = static_cast<int>(button);
    return (_checkChanged(button, current) && (current->buttons[index] != 0));
}

bool XboxController::_checkReleased(Button button, const sensor_msgs::Joy::ConstPtr& current) const {
    const int index = static_cast<int>(button);
    return (_checkChanged(button, current) && (current->buttons[index] == 0));
}

bool XboxController::_checkChanged(Button button, const sensor_msgs::Joy::ConstPtr& current) const {
    const int index = static_cast<int>(button);
    return (current->buttons[index] != _last.buttons[index]);
}

bool XboxController::_checkChanged(Analog input, const sensor_msgs::Joy::ConstPtr& current) const {
    const int index = static_cast<int>(input);
    return (current->axes[index] != _last.axes[index]);
}
