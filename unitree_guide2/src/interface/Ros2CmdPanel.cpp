/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS2_MB

#include "interface/Ros2CmdPanel.h"

#include <algorithm>
#include <cmath>
#include <functional>

Ros2CmdPanel::Ros2CmdPanel(rclcpp::Node::SharedPtr node) : _node(node){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    _inputMode = _node->declare_parameter<std::string>("input_mode", "keyboard");
    _joyTopic = _node->declare_parameter<std::string>("joy.topic", "/joy");
    _deadzone = _node->declare_parameter<double>("joy.deadzone", 0.08);
    _axisLx = _node->declare_parameter<int>("joy.axis_lx", 0);
    _axisLy = _node->declare_parameter<int>("joy.axis_ly", 1);
    _axisRx = _node->declare_parameter<int>("joy.axis_rx", 3);
    _axisRy = _node->declare_parameter<int>("joy.axis_ry", 4);
    _scaleLx = _node->declare_parameter<double>("joy.scale_lx", 1.0);
    _scaleLy = _node->declare_parameter<double>("joy.scale_ly", 1.0);
    _scaleRx = _node->declare_parameter<double>("joy.scale_rx", 1.0);
    _scaleRy = _node->declare_parameter<double>("joy.scale_ry", 1.0);
    _buttonA = _node->declare_parameter<int>("joy.button_a", 0);
    _buttonB = _node->declare_parameter<int>("joy.button_b", 1);
    _buttonX = _node->declare_parameter<int>("joy.button_x", 2);
    _buttonY = _node->declare_parameter<int>("joy.button_y", 3);
    _buttonL1 = _node->declare_parameter<int>("joy.button_l1", 4);
    _buttonL2 = _node->declare_parameter<int>("joy.button_l2", 6);
    _buttonStart = _node->declare_parameter<int>("joy.button_start", 7);
    _requireL2 = _node->declare_parameter<bool>("joy.require_l2", false);

    if(_inputMode != "keyboard" && _inputMode != "joystick" && _inputMode != "both"){
        RCLCPP_WARN(_node->get_logger(),
                    "Unknown input_mode '%s'; falling back to keyboard",
                    _inputMode.c_str());
        _inputMode = "keyboard";
    }

    if(_inputMode == "keyboard" || _inputMode == "both"){
        _keyboard = new KeyBoard();
        _activeSource = Source::KEYBOARD;
    }

    if(_inputMode == "joystick" || _inputMode == "both"){
        _joySub = _node->create_subscription<sensor_msgs::msg::Joy>(
            _joyTopic, rclcpp::SensorDataQoS(),
            std::bind(&Ros2CmdPanel::joyCallback, this, std::placeholders::_1));
        if(_inputMode == "joystick"){
            _activeSource = Source::JOYSTICK;
        }
    }

    RCLCPP_INFO(_node->get_logger(), "junior_ctrl input_mode: %s", _inputMode.c_str());
    if(_joySub){
        RCLCPP_INFO(_node->get_logger(), "Listening for joystick input on %s", _joyTopic.c_str());
    }
}

Ros2CmdPanel::~Ros2CmdPanel(){
    delete _keyboard;
}

UserCommand Ros2CmdPanel::getUserCmd(){
    refreshKeyboard();
    std::lock_guard<std::mutex> lock(_mutex);
    return _activeSource == Source::KEYBOARD ? _keyboardCmd : _joystickCmd;
}

UserValue Ros2CmdPanel::getUserValue(){
    refreshKeyboard();
    std::lock_guard<std::mutex> lock(_mutex);
    return _activeSource == Source::KEYBOARD ? _keyboardValue : _joystickValue;
}

void Ros2CmdPanel::setPassive(){
    std::lock_guard<std::mutex> lock(_mutex);
    _keyboardCmd = UserCommand::L2_B;
    _joystickCmd = UserCommand::L2_B;
    if(_keyboard){
        _keyboard->setPassive();
    }
}

void Ros2CmdPanel::setZero(){
    std::lock_guard<std::mutex> lock(_mutex);
    _keyboardValue.setZero();
    _joystickValue.setZero();
    if(_keyboard){
        _keyboard->setZero();
    }
}

void Ros2CmdPanel::refreshKeyboard(){
    if(!_keyboard){
        return;
    }

    const UserCommand command = _keyboard->getUserCmd();
    const UserValue value = _keyboard->getUserValue();

    std::lock_guard<std::mutex> lock(_mutex);
    const bool changed = command != _keyboardCmd || valuesDiffer(value, _keyboardValue);
    _keyboardCmd = command;
    _keyboardValue = value;
    if(changed && _inputMode == "both"){
        _activeSource = Source::KEYBOARD;
    }
}

void Ros2CmdPanel::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
    UserValue nextValue;
    nextValue.lx = readAxis(msg->axes, _axisLx, _scaleLx);
    nextValue.ly = readAxis(msg->axes, _axisLy, _scaleLy);
    nextValue.rx = readAxis(msg->axes, _axisRx, _scaleRx);
    nextValue.ry = readAxis(msg->axes, _axisRy, _scaleRy);
    nextValue.L2 = readButton(msg->buttons, _buttonL2) ? 1.0f : 0.0f;

    const bool l1 = readButton(msg->buttons, _buttonL1);
    const bool l2 = readButton(msg->buttons, _buttonL2);
    const bool a = readButton(msg->buttons, _buttonA);
    const bool b = readButton(msg->buttons, _buttonB);
    const bool x = readButton(msg->buttons, _buttonX);
    const bool y = readButton(msg->buttons, _buttonY);
    const bool start = readButton(msg->buttons, _buttonStart);
    const bool modeAllowed = !_requireL2 || l2;

    UserCommand nextCommand = UserCommand::NONE;
    bool commandPressed = false;
    if(x && l1){
        nextCommand = UserCommand::L1_X;
        commandPressed = true;
    }else if(a && l1){
        nextCommand = UserCommand::L1_A;
        commandPressed = true;
    }else if(y && l1){
        nextCommand = UserCommand::L1_Y;
        commandPressed = true;
    }else if(b && modeAllowed){
        nextCommand = UserCommand::L2_B;
        commandPressed = true;
    }else if(a && modeAllowed){
        nextCommand = UserCommand::L2_A;
        commandPressed = true;
    }else if(x && modeAllowed){
        nextCommand = UserCommand::L2_X;
        commandPressed = true;
    }else if(y && modeAllowed){
        nextCommand = UserCommand::L2_Y;
        commandPressed = true;
    }else if(start){
        nextCommand = UserCommand::START;
        commandPressed = true;
    }

    const bool stickActive = std::fabs(nextValue.lx) > 0.0f ||
                             std::fabs(nextValue.ly) > 0.0f ||
                             std::fabs(nextValue.rx) > 0.0f ||
                             std::fabs(nextValue.ry) > 0.0f;

    std::lock_guard<std::mutex> lock(_mutex);
    _joystickValue = nextValue;
    if(commandPressed){
        _joystickCmd = nextCommand;
    }
    if(_inputMode == "joystick" || commandPressed || stickActive ||
       _activeSource == Source::JOYSTICK){
        _activeSource = Source::JOYSTICK;
    }
}

float Ros2CmdPanel::readAxis(const std::vector<float>& axes, int index, double scale) const{
    if(index < 0 || static_cast<std::size_t>(index) >= axes.size()){
        return 0.0f;
    }
    double value = static_cast<double>(axes[index]) * scale;
    if(std::fabs(value) <= _deadzone){
        return 0.0f;
    }
    value = std::max(-1.0, std::min(1.0, value));
    return static_cast<float>(value);
}

bool Ros2CmdPanel::readButton(const std::vector<int32_t>& buttons, int index) const{
    return index >= 0 && static_cast<std::size_t>(index) < buttons.size() && buttons[index] != 0;
}

bool Ros2CmdPanel::valuesDiffer(const UserValue& lhs, const UserValue& rhs) const{
    return lhs.lx != rhs.lx || lhs.ly != rhs.ly || lhs.rx != rhs.rx ||
           lhs.ry != rhs.ry || lhs.L2 != rhs.L2;
}

#endif  // COMPILE_WITH_ROS2_MB
