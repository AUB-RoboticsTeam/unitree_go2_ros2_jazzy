/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"

class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void* runKeyBoard(void *arg);
    void* run(void *arg);
    UserCommand checkCmd();
    void changeValue();
    bool handleEscapeSequence();
    void adjustSpeed(float delta);

    pthread_t _tid;
    int _inputFd = -1;
    bool _threadStarted = false;
    bool _termiosConfigured = false;
    int _escapeState = 0;
    float sensitivityLeft = 0.05;
    float sensitivityRight = 0.05;
    float speedStep = 0.1;
    float minimumSpeedScale = 0.1;
    float maximumSpeedScale = 1.0;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    int ret;
    char _c;
};

#endif  // KEYBOARD_H
