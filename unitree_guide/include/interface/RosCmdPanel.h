/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ROSCMDPANEL_H
#define ROSCMDPANEL_H

#ifdef COMPILE_WITH_ROS

#include "interface/CmdPanel.h"
#include "interface/KeyBoard.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <mutex>
#include <string>
#include <vector>

// Keyboard and/or sensor_msgs/Joy input for junior_ctrl, selected with the
// private parameter ~input_mode (keyboard, joystick, or both).
class RosCmdPanel : public CmdPanel{
public:
    RosCmdPanel();
    ~RosCmdPanel() override;

    UserCommand getUserCmd() override;
    UserValue getUserValue() override;
    void setPassive() override;
    void setZero() override;

private:
    enum class Source { KEYBOARD, JOYSTICK };

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void refreshKeyboard();
    float readAxis(const std::vector<float>& axes, int index, double scale) const;
    bool readButton(const std::vector<int32_t>& buttons, int index) const;
    bool valuesDiffer(const UserValue& lhs, const UserValue& rhs) const;
    void adjustJoystickSpeed(double delta);

    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Subscriber _joySub;
    KeyBoard *_keyboard = nullptr;
    std::mutex _mutex;

    std::string _inputMode;
    std::string _joyTopic;
    Source _activeSource = Source::KEYBOARD;
    UserCommand _keyboardCmd = UserCommand::NONE;
    UserCommand _joystickCmd = UserCommand::NONE;
    UserValue _keyboardValue;
    UserValue _joystickValue;

    double _deadzone = 0.08;
    double _scaleLx = 1.0;
    double _scaleLy = 1.0;
    double _scaleRx = 1.0;
    double _scaleRy = 1.0;
    double _joystickSpeedLimit = 0.5;
    double _speedStep = 0.1;
    double _minimumSpeed = 0.1;
    double _maximumSpeed = 1.2;
    int _axisLx = 0;
    int _axisLy = 1;
    int _axisRx = 3;
    int _axisRy = 4;
    int _axisSpeed = 7;
    int _buttonA = 0;
    int _buttonB = 1;
    int _buttonX = 2;
    int _buttonY = 3;
    int _buttonL1 = 4;
    int _buttonL2 = 6;
    int _buttonStart = 7;
    int _buttonSpeedUp = 12;
    int _buttonSpeedDown = 14;
    bool _speedUpPressed = false;
    bool _speedDownPressed = false;
    bool _requireL2 = false;
};

#endif  // COMPILE_WITH_ROS
#endif  // ROSCMDPANEL_H
