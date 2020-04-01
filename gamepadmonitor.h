#ifndef GAMEPADMONITOR_H
#define GAMEPADMONITOR_H

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include "mainwindow.h"

QT_BEGIN_NAMESPACE
class QGamepad;
QT_END_NAMESPACE

class GamepadMonitor : public QObject
{
    Q_OBJECT
public:
    MainWindow* connectedWindow; // pointer to other window
    double maxDesiredB = 0.020; // [T] 15mT for stability, maximum 30mT realistic solutions
    bool enabled = false;
    QSerialPort calibration_port;
    explicit GamepadMonitor(QObject *parent = 0);
    ~GamepadMonitor();
    void enableController(void);
    void disableController(void);
    void associateWith(MainWindow* window);



private:
    QGamepad *m_gamepad;
    double joystickValues[6] = {0.0};
    void calibration_write(int value, int dir = 0, int axis = 0);
private slots:
    void leftXchanged( double value );
    void leftYchanged( double value );
    void rightXchanged( double value );
    void rightYchanged( double value );
    void buttonAchanged( bool pressed );
    void buttonBchanged( bool pressed );
    void buttonXchanged( bool pressed );
    void buttonYchanged( bool pressed );
    void buttonL1changed( bool pressed );
    void buttonR1changed( bool pressed );
    void buttonL2changed( double value );
    void buttonR2changed( double value );
    void buttonStartchanged( bool pressed );
    void buttonSelectchanged( bool pressed );
    void buttonGuidechanged( bool pressed );

    void setControllerState(bool state);
    void increaseBfield(void);

    void testfunction(int current_move, int step, int direction, int pin, float x_position, float y_position, float z_position);
    void smalltestfunction(int current_move, int step, int direction, int pin);

signals:
    void computeFieldSignal(void);
    void changeMagnetsSignal(void); // tells the program to write to arduino -> most computationally intense
    void changeRollSignal(int);
};


#endif // GAMEPADMONITOR_H
