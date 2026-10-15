/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/RosCmdPanel.h"

#include <algorithm>
#include <cmath>

RosCmdPanel::RosCmdPanel() : _pnh("~"){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    _pnh.param<std::string>("input_mode", _inputMode, "keyboard");
    _pnh.param<std::string>("joy/topic", _joyTopic, "/joy");
    _pnh.param("joy/deadzone", _deadzone, 0.08);
    _pnh.param("joy/axis_lx", _axisLx, 0);
    _pnh.param("joy/axis_ly", _axisLy, 1);
    _pnh.param("joy/axis_rx", _axisRx, 3);
    _pnh.param("joy/axis_ry", _axisRy, 4);
    _pnh.param("joy/scale_lx", _scaleLx, 1.0);
    _pnh.param("joy/scale_ly", _scaleLy, 1.0);
    _pnh.param("joy/scale_rx", _scaleRx, 1.0);
    _pnh.param("joy/scale_ry", _scaleRy, 1.0);
    _pnh.param("speed/initial_mps", _joystickSpeedLimit, 0.5);
    _pnh.param("speed/step_mps", _speedStep, 0.1);
    _pnh.param("speed/minimum_mps", _minimumSpeed, 0.1);
    _pnh.param("speed/maximum_mps", _maximumSpeed, 1.2);
    _joystickSpeedLimit = std::max(
        _minimumSpeed, std::min(_joystickSpeedLimit, _maximumSpeed));
    _pnh.param("joy/axis_speed", _axisSpeed, 7);
    _pnh.param("joy/button_a", _buttonA, 0);
    _pnh.param("joy/button_b", _buttonB, 1);
    _pnh.param("joy/button_x", _buttonX, 2);
    _pnh.param("joy/button_y", _buttonY, 3);
    _pnh.param("joy/button_l1", _buttonL1, 4);
    _pnh.param("joy/button_l2", _buttonL2, 6);
    _pnh.param("joy/button_start", _buttonStart, 7);
    _pnh.param("joy/button_speed_up", _buttonSpeedUp, 12);
    _pnh.param("joy/button_speed_down", _buttonSpeedDown, 14);
    _pnh.param("joy/require_l2", _requireL2, false);

    if(_inputMode != "keyboard" && _inputMode != "joystick" && _inputMode != "both"){
        ROS_WARN("Unknown input_mode '%s'; falling back to keyboard", _inputMode.c_str());
        _inputMode = "keyboard";
    }

    if(_inputMode == "keyboard" || _inputMode == "both"){
        _keyboard = new KeyBoard();
        _activeSource = Source::KEYBOARD;
    }

    if(_inputMode == "joystick" || _inputMode == "both"){
        _joySub = _nh.subscribe(_joyTopic, 10, &RosCmdPanel::joyCallback, this,
                                ros::TransportHints().tcpNoDelay());
        if(_inputMode == "joystick"){
            _activeSource = Source::JOYSTICK;
        }
    }

    ROS_INFO("junior_ctrl input_mode: %s", _inputMode.c_str());
    if(_joySub){
        ROS_INFO("Listening for joystick input on %s", _joyTopic.c_str());
    }
}

RosCmdPanel::~RosCmdPanel(){
    delete _keyboard;
}

UserCommand RosCmdPanel::getUserCmd(){
    refreshKeyboard();
    std::lock_guard<std::mutex> lock(_mutex);
    return _activeSource == Source::KEYBOARD ? _keyboardCmd : _joystickCmd;
}

UserValue RosCmdPanel::getUserValue(){
    refreshKeyboard();
    std::lock_guard<std::mutex> lock(_mutex);
    return _activeSource == Source::KEYBOARD ? _keyboardValue : _joystickValue;
}

void RosCmdPanel::setPassive(){
    std::lock_guard<std::mutex> lock(_mutex);
    _keyboardCmd = UserCommand::L2_B;
    _joystickCmd = UserCommand::L2_B;
    if(_keyboard){
        _keyboard->setPassive();
    }
}

void RosCmdPanel::setZero(){
    std::lock_guard<std::mutex> lock(_mutex);
    _keyboardValue.setZero();
    _joystickValue.setZero();
    if(_keyboard){
        _keyboard->setZero();
    }
}

void RosCmdPanel::refreshKeyboard(){
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

void RosCmdPanel::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    UserValue nextValue;
    nextValue.lx = readAxis(msg->axes, _axisLx, _scaleLx);
    nextValue.ly = readAxis(msg->axes, _axisLy, _scaleLy);
    nextValue.rx = readAxis(msg->axes, _axisRx, _scaleRx);
    nextValue.ry = readAxis(msg->axes, _axisRy, _scaleRy);
    nextValue.L2 = readButton(msg->buttons, _buttonL2) ? 1.0f : 0.0f;

    const float dpadVertical = readAxis(msg->axes, _axisSpeed, 1.0);
    const bool speedUp = readButton(msg->buttons, _buttonSpeedUp) || dpadVertical > 0.5f;
    const bool speedDown = readButton(msg->buttons, _buttonSpeedDown) || dpadVertical < -0.5f;
    if(speedUp && !_speedUpPressed){
        adjustJoystickSpeed(_speedStep);
    }
    if(speedDown && !_speedDownPressed){
        adjustJoystickSpeed(-_speedStep);
    }
    _speedUpPressed = speedUp;
    _speedDownPressed = speedDown;
    nextValue.speedLimit = static_cast<float>(_joystickSpeedLimit);

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
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(y && modeAllowed){
        nextCommand = UserCommand::L2_Y;
        commandPressed = true;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else if(start){
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

float RosCmdPanel::readAxis(const std::vector<float>& axes, int index, double scale) const{
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

bool RosCmdPanel::readButton(const std::vector<int32_t>& buttons, int index) const{
    return index >= 0 && static_cast<std::size_t>(index) < buttons.size() && buttons[index] != 0;
}

bool RosCmdPanel::valuesDiffer(const UserValue& lhs, const UserValue& rhs) const{
    return lhs.lx != rhs.lx || lhs.ly != rhs.ly || lhs.rx != rhs.rx ||
           lhs.ry != rhs.ry || lhs.L2 != rhs.L2 ||
           lhs.speedLimit != rhs.speedLimit;
}

void RosCmdPanel::adjustJoystickSpeed(double delta){
    _joystickSpeedLimit = std::max(
        _minimumSpeed,
        std::min(_joystickSpeedLimit + delta, _maximumSpeed));
    ROS_INFO("Maximum linear speed: %.1f m/s", _joystickSpeedLimit);
}

#endif  // COMPILE_WITH_ROS
