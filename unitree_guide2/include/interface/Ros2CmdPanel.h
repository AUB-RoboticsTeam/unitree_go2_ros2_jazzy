/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ROS2CMDPANEL_H
#define ROS2CMDPANEL_H

#ifdef COMPILE_WITH_ROS2_MB

#include "interface/CmdPanel.h"
#include "interface/KeyBoard.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <mutex>
#include <string>
#include <vector>

class Ros2CmdPanel : public CmdPanel{
public:
    explicit Ros2CmdPanel(rclcpp::Node::SharedPtr node);
    ~Ros2CmdPanel() override;

    UserCommand getUserCmd() override;
    UserValue getUserValue() override;
    void setPassive() override;
    void setZero() override;

private:
    enum class Source { KEYBOARD, JOYSTICK };

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void refreshKeyboard();
    float readAxis(const std::vector<float>& axes, int index, double scale) const;
    bool readButton(const std::vector<int32_t>& buttons, int index) const;
    bool valuesDiffer(const UserValue& lhs, const UserValue& rhs) const;
    void adjustJoystickSpeed(double delta);

    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySub;
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
    double _joystickSpeedScale = 0.5;
    double _speedStep = 0.1;
    double _minimumSpeedScale = 0.1;
    double _maximumSpeedScale = 1.0;
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

#endif  // COMPILE_WITH_ROS2_MB
#endif  // ROS2CMDPANEL_H
