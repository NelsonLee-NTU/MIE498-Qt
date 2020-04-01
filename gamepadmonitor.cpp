/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Gamepad module
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "gamepadmonitor.h"
#include "daq.h"
#include <QtGamepad/QGamepad>
#include <QWindow>
#include <QDebug>
#include <iostream>
#include <QApplication>
#include <math.h>

const int SPEED_FACTOR = 30;
const int COUNTS_PER_CYCLE = 2;
int RANGE = 90;
int SPACE = 45;
int STEP = RANGE/SPACE + 1;
long long int count = 0;
int move = 0;
int one_move = 8000;

bool calibration_is_running = false;
int calibration_countdown = 5;

int x_desired = 0;
int y_desired = 0;
int z_desired = 0;

int x_max = 20000;
int y_max = 100000;
int z_max = 100000;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

GamepadMonitor::GamepadMonitor(QObject *parent) : QObject(parent) , m_gamepad(0)
{
    QGamepadManager* gamepad_manager = QGamepadManager::instance();
    QList<int> gamepads;
    int i = 0;
    while (i < 10)
    {
        QApplication::processEvents();
        qInfo() << "get connected gamepads iteration : " << i;
        gamepads = gamepad_manager->connectedGamepads();
        if(!gamepads.isEmpty())
        {
            i = 10;
        }
        i++;
    }
    qInfo() << "connected gamepads : " << gamepads.size();
    m_gamepad = new QGamepad(*gamepads.begin(), this);
    // Create Gamepad object
    // Connect signals to slots:

    QTimer *changeFieldTimer = new QTimer(this);
    connect(changeFieldTimer, SIGNAL(timeout()), this, SLOT(increaseBfield()));
    changeFieldTimer->start(20); //Increase 50 times per second

    //enableController(); // Comment this out after testing purposes to default off on startup

    calibration_port.setPortName("COM9");
    calibration_port.setBaudRate(QSerialPort::Baud115200);
    calibration_port.setDataBits(QSerialPort::Data8);
    calibration_port.setParity(QSerialPort::NoParity);
    calibration_port.setStopBits(QSerialPort::OneStop);
    calibration_port.setFlowControl(QSerialPort::NoFlowControl);
    calibration_port.open(QIODevice::ReadWrite);

}

GamepadMonitor::~GamepadMonitor()
{
    delete m_gamepad;
}

void GamepadMonitor::associateWith(MainWindow* window)
{
    connectedWindow = window;
    connect(connectedWindow, &MainWindow::controllerStateSignal,  this, &GamepadMonitor::setControllerState);
    connect( this, SIGNAL(changeRollSignal(int)), connectedWindow, SLOT(changeRoll(int)) );
    connect( this, SIGNAL(changeMagnetsSignal()), connectedWindow, SLOT(updateMotors()) );
//    connect(connectedWindow->ui->comboBox_controllerMode,SIGNAL(currentIndexChanged (int)),this,SLOT( increaseBfield() ) );

}

void GamepadMonitor::enableController(void)
{
    // Unused buttons are commented out here and in disableController

    //connect(m_gamepad, &QGamepad::axisLeftXChanged,  this, &GamepadMonitor::leftXchanged);
    connect(m_gamepad, &QGamepad::axisLeftYChanged,  this, &GamepadMonitor::leftYchanged);
    connect(m_gamepad, &QGamepad::axisRightXChanged, this, &GamepadMonitor::rightXchanged);
    connect(m_gamepad, &QGamepad::axisRightYChanged, this, &GamepadMonitor::rightYchanged);
    connect(m_gamepad, &QGamepad::buttonAChanged,  this, &GamepadMonitor::buttonAchanged);
    connect(m_gamepad, &QGamepad::buttonBChanged,  this, &GamepadMonitor::buttonBchanged);
    connect(m_gamepad, &QGamepad::buttonXChanged,  this, &GamepadMonitor::buttonXchanged);
    connect(m_gamepad, &QGamepad::buttonYChanged,  this, &GamepadMonitor::buttonYchanged);
    connect(m_gamepad, &QGamepad::buttonL1Changed, this, &GamepadMonitor::buttonL1changed);
    connect(m_gamepad, &QGamepad::buttonR1Changed, this, &GamepadMonitor::buttonR1changed);
    connect(m_gamepad, &QGamepad::buttonL2Changed, this, &GamepadMonitor::buttonL2changed);
    connect(m_gamepad, &QGamepad::buttonR2Changed, this, &GamepadMonitor::buttonR2changed);
    //connect(m_gamepad, &QGamepad::buttonSelectChanged, this, &GamepadMonitor::buttonSelectchanged);
    connect(m_gamepad, &QGamepad::buttonStartChanged,  this, &GamepadMonitor::buttonStartchanged);
    //connect(m_gamepad, &QGamepad::buttonGuideChanged,  this, &GamepadMonitor::buttonGuidechanged);
    enabled = true;
}
void GamepadMonitor::disableController(void)
{
    //disconnect(m_gamepad, &QGamepad::axisLeftXChanged,  this, &GamepadMonitor::leftXchanged);
    disconnect(m_gamepad, &QGamepad::axisLeftYChanged,  this, &GamepadMonitor::leftYchanged);
    disconnect(m_gamepad, &QGamepad::axisRightXChanged, this, &GamepadMonitor::rightXchanged);
    disconnect(m_gamepad, &QGamepad::axisRightYChanged, this, &GamepadMonitor::rightYchanged);
    disconnect(m_gamepad, &QGamepad::buttonAChanged,  this, &GamepadMonitor::buttonAchanged);
    //disconnect(m_gamepad, &QGamepad::buttonBChanged,  this, &GamepadMonitor::buttonBchanged);
    disconnect(m_gamepad, &QGamepad::buttonXChanged,  this, &GamepadMonitor::buttonXchanged);
    //disconnect(m_gamepad, &QGamepad::buttonYChanged,  this, &GamepadMonitor::buttonYchanged);
    disconnect(m_gamepad, &QGamepad::buttonL1Changed, this, &GamepadMonitor::buttonL1changed);
    disconnect(m_gamepad, &QGamepad::buttonR1Changed, this, &GamepadMonitor::buttonR1changed);
    disconnect(m_gamepad, &QGamepad::buttonL2Changed, this, &GamepadMonitor::buttonL2changed);
    disconnect(m_gamepad, &QGamepad::buttonR2Changed, this, &GamepadMonitor::buttonR2changed);
    //disconnect(m_gamepad, &QGamepad::buttonSelectChanged, this, &GamepadMonitor::buttonSelectchanged);
    disconnect(m_gamepad, &QGamepad::buttonStartChanged,  this, &GamepadMonitor::buttonStartchanged);
    //disconnect(m_gamepad, &QGamepad::buttonGuideChanged,  this, &GamepadMonitor::buttonGuidechanged);
    enabled = false;
    for (int i=0;i<4;i++)
    {
        joystickValues[i] = 0.0;
    }
}

void GamepadMonitor::calibration_write(int value, int dir, int axis)
{
    //QByteArray output =  QByteArray::number(value) + QByteArray::number(dir) + QByteArray::number(axis);
    if(x_desired < x_max && y_desired < y_max && z_desired < z_max) {
        switch(axis) {
            case 0:
                if(dir == 0)
                    y_desired -= value;
                else if(dir == 1)
                    y_desired += value;
                break;
            case 1:
                if(dir == 0)
                    x_desired -= value;
                else if(dir == 1)
                    x_desired += value;
                break;
            case 2:
                if(dir == 0)
                    z_desired -= value;
                else if(dir == 1)
                    z_desired += value;
                break;
        }

        std::string outputString = std::to_string(value) + std::to_string(dir) + std::to_string(axis) + ",";
        QByteArray outputdata(outputString.c_str(),outputString.length());
        calibration_port.write(outputdata);
        qDebug() << outputString.c_str();
    }
//    std::string outputString = std::to_string(value) + std::to_string(dir) + std::to_string(axis) + ",";
//    if(count == 1)
//    {
//        calibration_port.setPortName("COM9");
//        calibration_port.setBaudRate(QSerialPort::Baud115200);
//        calibration_port.setDataBits(QSerialPort::Data8);
//        calibration_port.setParity(QSerialPort::NoParity);
//        calibration_port.setStopBits(QSerialPort::OneStop);
//        calibration_port.setFlowControl(QSerialPort::NoFlowControl);
//        calibration_port.open(QIODevice::ReadWrite);
//    }
//    QByteArray outputdata(outputString.c_str(),outputString.length());
//    calibration_port.write(outputdata);
//    qDebug() << outputString.c_str();
}

// ------------------------------------- SLOTS -------------------------------------//

void GamepadMonitor::leftXchanged( double value )
{
//    qDebug() << "Left X" << value;
    // For states:
    // 0. nothing
    // 1. nothing
    // 2. nothing
    // 3. nothing
    joystickValues[4] = value * abs(value);
}

void GamepadMonitor::leftYchanged( double value )
{
//    qDebug() << "Left Y" << value;

    // For states:
    // 0. Bx value increasing nonlinearly
    // 1. Change in Bx value increasing nonlinearly
    // 2. Change in Phi value increasing nonlinearly
    // 3. Tool In/Out
    joystickValues[0] = value * abs(value);
}

void GamepadMonitor::rightXchanged( double value )
{
    // Tool in
//    qDebug() << "Right X" << value;
    //Debugging
    connectedWindow->joystickVal = value * abs(value); // Increase nonlinearly to add sensitivity and control

    // For states:
    // 0. By value increasing nonlinearly
    // 1. Change in By value increasing nonlinearly
    // 2. Change in Theta value increasing nonlinearly
    // 3. Change in By value increasing nonlinearly

    joystickValues[1] = value * abs(value); // Theta value increasing nonlinearly

}

void GamepadMonitor::rightYchanged( double value )
{
//    qDebug() << "Right Y" << value;
    // For states:
    // 0. nothing
    // 1. nothing
    // 2. nothing
    // 3. Change in Bx value increasing nonlinearly

    joystickValues[5] = value * abs(value);
}

void GamepadMonitor::buttonAchanged( bool pressed )
{
    // Tool out fast
//    qDebug() << "Button A" << pressed;
    if (pressed)
    {
        connectedWindow->ToolOUT_press_Slot();
    }
    else
    {
        connectedWindow->ToolINOUT_rel_Slot();
    }

}

void GamepadMonitor::buttonBchanged( bool pressed )
{
    qDebug() << "Button B" << pressed;
    if (pressed)
    {
        // zeroing
//        calibration_write(1000,1,3);
        calibration_write(1000,1,3);
        x_desired = 0;
        y_desired = 0;
        z_desired = 0;
    }
}

void GamepadMonitor::buttonXchanged( bool pressed )
{
    // Tool in fast
    qDebug() << "Button X" << pressed;
    if (pressed)
    {
        connectedWindow->ToolIN_press_Slot();
    }
    else
    {
        connectedWindow->ToolINOUT_rel_Slot();
    }

}

void GamepadMonitor::buttonYchanged( bool pressed )
{
    qDebug() << "Button Y" << pressed;
    // Show the current controller state in terminal
    qDebug() << connectedWindow->currentControllerMode;
//    calibration_write(1000,1,0);
//    calibration_write(1000,1,1);
//    calibration_write(500,1,2);
//    calibration_write(1000,0,0);
//    calibration_write(1000,0,1);


    if (pressed)
    {
        calibration_is_running = true;
    }


}

void GamepadMonitor::buttonL1changed( bool pressed )
{
    qDebug() << "Button L1" << pressed;
    if ( pressed )
    {
        connectedWindow->roll_dir = -1;
        emit changeRollSignal(-1);
    }
    else
    {
        connectedWindow->roll_dir = 0;
        emit changeRollSignal(0);
    }

}

void GamepadMonitor::buttonR1changed( bool pressed )
{
    qDebug() << "Button R1" << pressed;
    if ( pressed )
    {
        connectedWindow->roll_dir = 1;
        emit changeRollSignal(1);
    }
    else
    {
        connectedWindow->roll_dir = 0;
        emit changeRollSignal(0);
    }

}

void GamepadMonitor::buttonL2changed( double value )
{
//    qDebug() << "Button L2: " << value;

    // For Controller states:
    // 0. Bz value decreasing nonlinearly (negative direction)
    // 1. Change in Bz value decreasing nonlinearly (negative direction)
    // 2. Change in B_mag value decreasing nonlinearly (negative direction)
    // 3. Change in Bz value decreasing nonlinearly (negative direction)

//    joystickValues[2] = value * abs(value);
//    calibration_write(1000,0,0);
    if(value == 1) {
        qDebug() << "Go to zeroing point";
        calibration_write(1000,0,5);
    }

}

void GamepadMonitor::buttonR2changed( double value )
{
//    qDebug() << "Button R2: " << value;

    // For states:
    // 0. Bz value increasing nonlinearly
    // 1. Change in Bz value increasing nonlinearly
    // 2. Change in B_mag value increasing nonlinearly
    // 3. Change in Bz value increasing nonlinearly

    joystickValues[3] = value * abs(value);
}

void GamepadMonitor::buttonStartchanged( bool pressed )
{
    qDebug() << "Button Start" << pressed;
}
void GamepadMonitor::buttonSelectchanged( bool pressed )
{
    qDebug() << "Button Select" << pressed;
}

void GamepadMonitor::buttonGuidechanged( bool pressed )
{
    qDebug() << "Button Guide" << pressed;
}

void GamepadMonitor::setControllerState(bool state)
{
    if (state)
    {
        enableController();
    }
    else
    {
        disableController();
    }
}

void GamepadMonitor::increaseBfield(void) // Callback for updating motor position
{

    if (connectedWindow->currentControllerMode == 0 && enabled) // Update DIRECT in B-field
    {
        // Set desired B local field
        connectedWindow->B_Desired[0] = -this->maxDesiredB*joystickValues[0] ; // Increase nonlinearly to add sensitivity and control
        connectedWindow->B_Desired[1] =  this->maxDesiredB*joystickValues[1] ; // Increase nonlinearly to add sensitivity and control
        connectedWindow->B_Desired[2] = -this->maxDesiredB*joystickValues[2] + this->maxDesiredB*joystickValues[3]; // Increase nonlinearly to add sensitivity and control
        // Set desired global field
        connectedWindow->tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
        // Find best motor angle solution to desired field
        connectedWindow->find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
        // Determine the resulting actual field
        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
        // Write angles to motors
        emit changeMagnetsSignal();
        connectedWindow->updateAngles();
    }
    else if (connectedWindow->currentControllerMode == 1 && enabled) // Update Change in B-field
    {
        // Set desired Bz local field
        if (abs(joystickValues[0]) > 0.075) // threshold because of the noisy controller in x
        {
            connectedWindow->B_Desired[0] = connectedWindow->B_Desired[0] + -this->maxDesiredB*joystickValues[0]/100 ; // Increase nonlinearly to add sensitivity and control
        }
        connectedWindow->B_Desired[1] = connectedWindow->B_Desired[1] + this->maxDesiredB*joystickValues[1]/100 ; // Increase nonlinearly to add sensitivity and control
        connectedWindow->B_Desired[2] = connectedWindow->B_Desired[2] - this->maxDesiredB*joystickValues[2]/100 + this->maxDesiredB*joystickValues[3]/100; // Increase nonlinearly to add sensitivity and control

        // Threshold the max Bfield so it doesn't increase to an absurd number
        for (int i=0;i<3;i++)
        {
            if (abs(connectedWindow->B_Desired[i]) > this->maxDesiredB)
            {
                connectedWindow->B_Desired[i] = this->maxDesiredB * connectedWindow->B_Desired[i]/abs(connectedWindow->B_Desired[i]);
            }
        }
        // Set desired global field
        connectedWindow->tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
        // Find best motor angle solution to desired field
        connectedWindow->find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
        // Determine the resulting actual field
        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
        // Write angles to motors
        emit changeMagnetsSignal();
        connectedWindow->updateAngles();
    }
    else if (connectedWindow->currentControllerMode == 2 && enabled) // Update Change in Polar Direction
    {
        if (abs(joystickValues[0]) > 0.075) // threshold because of the noisy controller in x
        {
            connectedWindow->angle2   = connectedWindow->angle2   + joystickValues[0] / 25; //phi
        }
        connectedWindow->angle1 = connectedWindow->angle1 + joystickValues[1] / 25;
        connectedWindow->B_mag = connectedWindow->B_mag - joystickValues[2] / 5000 + joystickValues[3] / 5000;
        // Threshold the max Bfield so it doesn't increase to an absurd number
        if ( abs(connectedWindow->B_mag) > this->maxDesiredB)
        {
            connectedWindow->B_mag = this->maxDesiredB * connectedWindow->B_mag/abs(connectedWindow->B_mag); // x/abs(x) = sign(x);
        }
        connectedWindow->B_Desired[0] = connectedWindow->B_mag*sin(connectedWindow->angle2)*cos(connectedWindow->angle1); // use phi and theta, angles 1 and 2
        connectedWindow->B_Desired[1] = connectedWindow->B_mag*sin(connectedWindow->angle2)*sin(connectedWindow->angle1); // use phi and theta, angles 1 and 2
        connectedWindow->B_Desired[2] = connectedWindow->B_mag*cos(connectedWindow->angle2); //
        // Set desired global field
        connectedWindow->tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
        // Find best motor angle solution to desired field
        connectedWindow->find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
        // Determine the resulting actual field
        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
        // Write angles to motors
        emit changeMagnetsSignal();
        connectedWindow->updateAngles();
    }
    else if (connectedWindow->currentControllerMode == 3 && enabled) // Surgeon Simulator
    {
        // Set desired In/Out Speed of tool
        if (abs(joystickValues[0]) > 0.075) // threshold because of the noisy controller in x
        {
//            joystickValues[0] = (abs(joystickValues[0]) - 0.075) * (joystickValues[0]/abs(joystickValues[0])); // subtract the dead zone
            connectedWindow->ToolINOUT_speed_Slot(int(-100*sqrt(abs(joystickValues[0])) * (joystickValues[0]/abs(joystickValues[0]))) + 110); // Value -1 to 1 shifted to 10 to 210
        }
        else
        {
            connectedWindow->ToolINOUT_rel_Slot(); // sends a stop command
            // This doesn't allow the a faster in/out motion with the buttons
        }

        // Check If Field control or Angle control is Enabled
        if (connectedWindow->enableAngleControlState)
        {
            connectedWindow->angle1   = connectedWindow->angle1   - joystickValues[5] / 60; // change - to + to invert direction  of joystick
            connectedWindow->angle2   = connectedWindow->angle2   + joystickValues[1] / 60;
            connectedWindow->B_mag = connectedWindow->B_mag - this->maxDesiredB*joystickValues[2]/100 + this->maxDesiredB*joystickValues[3]/100; // Increase nonlinearly to add sensitivity and control
            // Check bounds
            if (abs(connectedWindow->angle1) > M_PI/2)
            {
                connectedWindow->angle1 = M_PI/2 * connectedWindow->angle1/abs(connectedWindow->angle1); // (keeps the polarity of the angle)
            }
            if (abs(connectedWindow->angle2) > M_PI/2)
            {
                connectedWindow->angle2 = M_PI/2 * connectedWindow->angle2/abs(connectedWindow->angle2); // (keeps the polarity of the angle)
            }

            connectedWindow->calculateTheta();
            connectedWindow->findB_SurgeonSimulator(connectedWindow->B_Desired);
        }
        else
        {
            connectedWindow->angle1 = 0.0;
            connectedWindow->angle2 = 0.0;
            connectedWindow->B_mag = 0.0;
            connectedWindow->B_Desired[0] = connectedWindow->B_Desired[0] + -this->maxDesiredB*joystickValues[5]/100 ; // Increase nonlinearly to add sensitivity and control
            connectedWindow->B_Desired[1] = connectedWindow->B_Desired[1] + this->maxDesiredB*joystickValues[1]/100 ; // Increase nonlinearly to add sensitivity and control
            connectedWindow->B_Desired[2] = connectedWindow->B_Desired[2] - this->maxDesiredB*joystickValues[2]/100 + this->maxDesiredB*joystickValues[3]/100; // Increase nonlinearly to add sensitivity and control
        }

        // Threshold the max Bfield so it doesn't increase to an absurd number
        for (int i=0;i<3;i++)
        {
            if (abs(connectedWindow->B_Desired[i]) > this->maxDesiredB)
            {
                connectedWindow->B_Desired[i] = this->maxDesiredB * connectedWindow->B_Desired[i]/abs(connectedWindow->B_Desired[i]);
            }
        }
        // Set desired global field
        connectedWindow->tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
        // Find best motor angle solution to desired field
        connectedWindow->find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
        // Determine the resulting actual field
        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
        // Write angles to motors
        emit changeMagnetsSignal();
        connectedWindow->updateAngles();
    }
    else if (connectedWindow->currentControllerMode == 4 && enabled) //Automated Calibration
    {
        //Increment of angles of each motor by 30 degrees each in cycles
        for (int i=0;i<4 && count==0;i++)
            connectedWindow->init_Angles[i] = 0;
        if (count<pow(COUNTS_PER_CYCLE,4)*SPEED_FACTOR)
        {
            count+=1;
            if(count%((long long int)pow(COUNTS_PER_CYCLE,4)*SPEED_FACTOR) == 0)
                connectedWindow->init_Angles[0] += M_PI/6;
            else if(count%((long long int)pow(COUNTS_PER_CYCLE,3)*SPEED_FACTOR) == 0)
                connectedWindow->init_Angles[1] += M_PI/6;
            else if(count%((long long int)pow(COUNTS_PER_CYCLE,2)*SPEED_FACTOR) == 0)
                connectedWindow->init_Angles[2] += M_PI/6;
            else if(count%(COUNTS_PER_CYCLE*SPEED_FACTOR) == 0)
                connectedWindow->init_Angles[3] += M_PI/6;
            for (int i=0;i<4;i++)
                if(connectedWindow->init_Angles[i]>2*M_PI)
                    connectedWindow->init_Angles[i] -= 2*M_PI;
            qDebug()<<"count "<<count<<connectedWindow->init_Angles[0]*180/M_PI<<" "<<connectedWindow->init_Angles[1]*180/M_PI<<" "<<connectedWindow->init_Angles[2]*180/M_PI<<" "<<connectedWindow->init_Angles[3]*180/M_PI;
            if (count%(COUNTS_PER_CYCLE*SPEED_FACTOR) == (COUNTS_PER_CYCLE*SPEED_FACTOR)/2)
//                data_acquisition(connectedWindow->init_Angles);
            emit changeMagnetsSignal();
            //joystickValues[0]
            //calibration_write(1000,1,2);
            //calibration_write(abs(joystickValues[0])*1000,0,2);

        }
    }
    else if (connectedWindow->currentControllerMode == 5 && enabled) //Manual CNC Control
    {
//        qDebug() << x_desired << " " << y_desired << " " << z_desired;
        if (calibration_is_running == true)
        {
//          count+=1;
          const int time = 100;

          switch(move) {
            case 0:
              testfunction(0,0,0,0,0,0,0);
              qDebug() << "move = 0";
              break;

            case 1:
              testfunction(1,1000,0,1,0,1,0);
              qDebug() << "move =1";
              break;

            case 2:
              testfunction(2,1000,1,0,-1,1,0);
              qDebug() << "move =2";
              break;

            case 3:
              testfunction(3,1000,1,1,-1,0,0);
              qDebug() << "move = 3";
              break;
            case 4:
              testfunction(4,1000,1,1,-1,-1,0);
              qDebug() << "move = 4";
              break;

            case 5:
              testfunction(5,1000,0,0,0,-1,0);
              qDebug() << "move = 5";
              break;
            case 6:
              testfunction(6,1000,0,0,1,-1,0);
              qDebug() << "move = 6";
              break;
            case 7:
              testfunction(7,1000,0,1,1,0,0);
              qDebug() << "move = 7";
              break;
            case 8:
              testfunction(8,1000,0,1,1,1,0);
              qDebug() << "move = 8";
              break;

            case 7*time:
              calibration_write(10000,1,0);
              break;
            case 8*time:
              calibration_write(10000,1,0);
              break;
            case 9*time:
              calibration_write(10000,1,0);
              break;
            case 10*time:
              calibration_write(10000,1,0);
              break;
            case 11*time:
              calibration_write(10000,1,0);
              count = 0;
              calibration_is_running = false;
              break;
          }

        }
    }

    else if (connectedWindow->currentControllerMode == 6 && enabled) //Manual CNC Control
    {
        if (calibration_is_running == true) {
            switch(move) {
                case 0:
                    smalltestfunction(0,one_move,0,1); //(0,0,0)
                    break;
                case 1:
                    smalltestfunction(1,one_move,1,0); //(-1,0,0)
                    qDebug() << "move = 1";
                    break;
                case 2:
                    smalltestfunction(2,one_move,1,1); //(-1,1,0)
                    qDebug() << "move = 2";
                    break;
                case 3:
                    smalltestfunction(3,one_move,1,1); //(0,1,0)
                  qDebug() << "move = 3";
                  break;
                case 4:
                    smalltestfunction(4,one_move,0,0); //(1,1,0)
                  qDebug() << "move = 4";
                  break;
                case 5:
                    smalltestfunction(5,one_move,0,0); //(1,0,0)
                  qDebug() << "move = 5";
                  break;
                case 6:
                    smalltestfunction(6,one_move,0,1); //(1,-1,0)
                  qDebug() << "move = 6";
                  break;
                case 7:
                    smalltestfunction(7,one_move,0,1); //(0,-1,0)
                  qDebug() << "move = 7";
                  break;
                case 8:
                    smalltestfunction(8,one_move,1,2); //(-1,-1,0)
                  qDebug() << "move = 8";
                  break;

                case 9:
                    smalltestfunction(9,one_move,1,0); //(-1,-1,1)
                    qDebug() << "move =9";
                    break;
                case 10:
                    smalltestfunction(10,one_move,1,0); //(-1,0,1)
                    qDebug() << "move =10";
                    break;
                case 11:
                    smalltestfunction(11,one_move,1,1); //(-1,1,1)
                  qDebug() << "move = 11";
                  break;
                case 12:
                    smalltestfunction(12,one_move,1,1); //(0,1,1)
                  qDebug() << "move = 12";
                  break;
                case 13:
                    smalltestfunction(13,one_move,0,0); //(1,1,1)
                  qDebug() << "move = 13";
                  break;
                case 14:
                    smalltestfunction(14,one_move,0,0); //(1,0,1)
                  qDebug() << "move = 14";
                  break;
                case 15:
                    smalltestfunction(15,one_move,0,1); //(1,-1,1)
                  qDebug() << "move = 15";
                  break;
                case 16:
                    smalltestfunction(16,one_move,1,0); //(0,-1,1)
                  qDebug() << "move = 16";
                  break;
                case 17:
                    smalltestfunction(17,one_move,1,2); //(0,0,1)
                  qDebug() << "move = 17";
                  break;

                case 18:
                    smalltestfunction(18,one_move,0,1); //(0,0,2)
                    qDebug() << "move = 18";
                    break;
                case 19:
                    smalltestfunction(19,one_move,1,0); //(-1,0,2)
                    qDebug() << "move = 19";
                    break;
                case 20:
                    smalltestfunction(20,one_move,1,1); //(-1,1,2)
                    qDebug() << "move =20";
                    break;
                case 21:
                    smalltestfunction(21,one_move,1,1); //(0,1,2)
                  qDebug() << "move = 21";
                  break;
                case 22:
                    smalltestfunction(22,one_move,0,0); //(1,1,2)
                  qDebug() << "move = 22";
                  break;
                case 23:
                    smalltestfunction(23,one_move,0,0); //(1,0,2)
                  qDebug() << "move = 23";
                  break;
                case 24:
                    smalltestfunction(24,one_move,0,1); //(1,-1,2)
                  qDebug() << "move = 24";
                  break;
                case 25:
                    smalltestfunction(25,one_move,0,1); //(0,-1,2)
                  qDebug() << "move = 25";
                  break;
                case 26:
                    smalltestfunction(26,0,1,2); //(-1,-1,2)
                  qDebug() << "done";
                  break;
            }

        }

    }

    else if (connectedWindow->currentControllerMode == 7 && enabled) //Manual CNC Control
    {
        for (int i=0;i<4;i++){
                connectedWindow->init_Angles[i] = 0;
                connectedWindow->init_Angles[i] += 0;
                emit changeMagnetsSignal();
        }

    }

}

void GamepadMonitor::testfunction(int current_move, int step, int direction, int pin, float x_position, float y_position, float z_position) {
    for (int i=0;i<4 && count==1;i++)
        connectedWindow->init_Angles[i] = 0;

    if (count<2*pow(COUNTS_PER_CYCLE,4)*SPEED_FACTOR)
    {
        count+=1;
        if(count%((long long int)pow(COUNTS_PER_CYCLE,4)*SPEED_FACTOR) == 0)
            connectedWindow->init_Angles[0] += 2*M_PI/COUNTS_PER_CYCLE;
        if(count%((long long int)pow(COUNTS_PER_CYCLE,3)*SPEED_FACTOR) == 0)
            connectedWindow->init_Angles[1] += 2*M_PI/COUNTS_PER_CYCLE;
        if(count%((long long int)pow(COUNTS_PER_CYCLE,2)*SPEED_FACTOR) == 0)
            connectedWindow->init_Angles[2] += 2*M_PI/COUNTS_PER_CYCLE;
        if(count%(COUNTS_PER_CYCLE*SPEED_FACTOR) == 0)
            connectedWindow->init_Angles[3] += 2*M_PI/COUNTS_PER_CYCLE;
        for (int i=0;i<4;i++)
            if(connectedWindow->init_Angles[i]>=2*M_PI)
                connectedWindow->init_Angles[i] -= 2*M_PI;
        qDebug()<< "count "<< count << connectedWindow->init_Angles[0]*180/M_PI <<" "<<connectedWindow->init_Angles[1]*180/M_PI<<" "<<connectedWindow->init_Angles[2]*180/M_PI<<" "<<connectedWindow->init_Angles[3]*180/M_PI;
        if (count%(COUNTS_PER_CYCLE*SPEED_FACTOR) == (COUNTS_PER_CYCLE*SPEED_FACTOR)/2)
            data_acquisition(connectedWindow->init_Angles, x_position, y_position, z_position);
        emit changeMagnetsSignal();
    }

    else {
        if(count == 2*pow(COUNTS_PER_CYCLE,4)*SPEED_FACTOR) calibration_write(step,direction,pin);
        count += 1;
        if(count == (2*pow(COUNTS_PER_CYCLE,4)*SPEED_FACTOR + 200)) {
            count = 0;
            move = current_move + 1;
        }
    }
}

void GamepadMonitor::smalltestfunction(int current_move, int step, int direction, int pin) {
    qDebug() << x_desired << " " << y_desired << " " << z_desired;
    for (int i=0;i<4;i++){
        if(count == 1) {
            connectedWindow->init_Angles[i] = 0;
            connectedWindow->init_Angles[i] += 0;
        }
    }


    if (count<((long long int)pow(STEP,4)+1)*SPEED_FACTOR)
    {
        count+=1;

        if(count%(SPEED_FACTOR) == 0){
            connectedWindow->init_Angles[3] += SPACE*M_PI/180;
        }
        if(count%(STEP*SPEED_FACTOR) == 0) {
            connectedWindow->init_Angles[2] += SPACE*M_PI/180;
        }
        if(count%((long long int)pow(STEP,2)*SPEED_FACTOR) == 0) {
            connectedWindow->init_Angles[1] += SPACE*M_PI/180;
        }
        if(count%((long long int)pow(STEP,3)*SPEED_FACTOR) == 0) {
            connectedWindow->init_Angles[0] += SPACE*M_PI/180;
        }

        for (int i=0;i<4;i++)
            if(connectedWindow->init_Angles[i]>=0.75*M_PI)
                connectedWindow->init_Angles[i] -= 0.75*M_PI;

//        for (int i=0;i<4;i++){
//            if(connectedWindow->init_Angles[i] >= 90/180*M_PI) {
//                connectedWindow->init_Angles[i] -= 90/180*M_PI;
//            }
//        }

        qDebug()<< "count "<< count << connectedWindow->init_Angles[0]*180/M_PI <<" "<<connectedWindow->init_Angles[1]*180/M_PI<<" "<<connectedWindow->init_Angles[2]*180/M_PI<<" "<<connectedWindow->init_Angles[3]*180/M_PI;
        if (count%(SPEED_FACTOR) == (SPEED_FACTOR)/2){
            data_acquisition(connectedWindow->init_Angles, x_desired, y_desired, z_desired);
            qDebug() << "daq";
        }
        emit changeMagnetsSignal();
    }

    else {
        if(count == ((long long int)pow(STEP,4)+1)*SPEED_FACTOR) {
            calibration_write(step,direction,pin);
        }
        count += 1;
        if(count == (((long long int)pow(STEP,4)+1)*SPEED_FACTOR + 100)) {
            count = 0;
            move = current_move + 1;
            qDebug() << "done";
        }
    }


//    if (count<STEP*SPEED_FACTOR)
//    {
//        count+=1;

//        if(count%(SPEED_FACTOR) == 0){
//            for (int i=0; i<4; i++)
//                connectedWindow->init_Angles[i] += SPACE*M_PI/180;
//        }
//        for (int i=0;i<4;i++)
//            if(connectedWindow->init_Angles[i]>=2*M_PI)
//                connectedWindow->init_Angles[i] -= 2*M_PI;

//        qDebug()<< "count "<< count << connectedWindow->init_Angles[0]*180/M_PI <<" "<<connectedWindow->init_Angles[1]*180/M_PI<<" "<<connectedWindow->init_Angles[2]*180/M_PI<<" "<<connectedWindow->init_Angles[3]*180/M_PI;
//        if (count%(SPEED_FACTOR) == (SPEED_FACTOR)/2){
//            data_acquisition(connectedWindow->init_Angles, x_desired, y_desired, z_desired);
//            qDebug() << "daq";
//        }
//        emit changeMagnetsSignal();
//    }

//    else {
//        if(count == STEP*SPEED_FACTOR) {
//            calibration_write(step,direction,pin);
//        }
//        count += 1;
//        if(count == (STEP*SPEED_FACTOR + 100)) {
//            count = 0;
//            move = current_move + 1;
//            qDebug() << "done";
//        }
//    }

}

