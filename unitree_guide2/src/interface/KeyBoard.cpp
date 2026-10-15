/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <cerrno>
#include <cstring>
#include <iostream>

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    // ROS 2 launch connects a node's stdin to a pipe. Open the controlling
    // terminal explicitly so keyboard control works both with `ros2 run` and
    // `ros2 launch`.
    _inputFd = open("/dev/tty", O_RDWR | O_CLOEXEC);
    if(_inputFd < 0 && isatty(STDIN_FILENO)){
        _inputFd = dup(STDIN_FILENO);
    }
    if(_inputFd < 0){
        std::cerr << "Keyboard input unavailable: no controlling terminal ("
                  << std::strerror(errno) << ")" << std::endl;
        return;
    }

    if(tcgetattr(_inputFd, &_oldSettings) != 0){
        std::cerr << "Keyboard input unavailable: tcgetattr failed ("
                  << std::strerror(errno) << ")" << std::endl;
        close(_inputFd);
        _inputFd = -1;
        return;
    }
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    if(tcsetattr(_inputFd, TCSANOW, &_newSettings) != 0){
        std::cerr << "Keyboard input unavailable: tcsetattr failed ("
                  << std::strerror(errno) << ")" << std::endl;
        close(_inputFd);
        _inputFd = -1;
        return;
    }
    _termiosConfigured = true;

    const int threadError = pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
    if(threadError != 0){
        std::cerr << "Keyboard input unavailable: pthread_create failed ("
                  << std::strerror(threadError) << ")" << std::endl;
        tcsetattr(_inputFd, TCSANOW, &_oldSettings);
        _termiosConfigured = false;
        close(_inputFd);
        _inputFd = -1;
        return;
    }
    _threadStarted = true;
}

KeyBoard::~KeyBoard(){
    if(_threadStarted){
        pthread_cancel(_tid);
        pthread_join(_tid, NULL);
    }
    if(_termiosConfigured){
        tcsetattr(_inputFd, TCSANOW, &_oldSettings);
    }
    if(_inputFd >= 0){
        close(_inputFd);
    }
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
#ifdef COMPILE_WITH_ROS2_MB
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_ROS2_MB
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

bool KeyBoard::handleEscapeSequence(){
    if(_escapeState == 0){
        if(_c == '\x1b'){
            _escapeState = 1;
            return true;
        }
        return false;
    }

    if(_escapeState == 1){
        if(_c == '[' || _c == 'O'){
            _escapeState = 2;
            return true;
        }
        _escapeState = 0;
        return true;
    }

    if(_c == 'A'){
        adjustSpeed(speedStep);
    }else if(_c == 'B'){
        adjustSpeed(-speedStep);
    }
    _escapeState = 0;
    return true;
}

void KeyBoard::adjustSpeed(float delta){
    userValue.speedScale = max<float>(
        minimumSpeedScale,
        min<float>(userValue.speedScale + delta, maximumSpeedScale));
    std::cout << "Speed scale: "
              << static_cast<int>(userValue.speedScale * 100.0f + 0.5f)
              << "%" << std::endl;
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET(_inputFd, &set);

        res = select(_inputFd + 1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read(_inputFd, &_c, 1);
            if(ret == 1){
                if(!handleEscapeSequence()){
                    userCmd = checkCmd();
                    if(userCmd == UserCommand::NONE)
                        changeValue();
                }
                _c = '\0';
            }
        }else if(res < 0 && errno != EINTR){
            break;
        }
        usleep(1000);
    }
    return NULL;
}
