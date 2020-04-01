#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <fstream>
#include <QtSerialPort/QSerialPort>
//#include "qcustomplot.h"

//#include "gamepadmonitor.h"

const double mu_0 = 3.14159265359 * 4e-7;
const double Br = 1.457; // For Gripper experiments: 1.47; // [T]
const double M = 1/mu_0*Br;
const double vAct = 0.0508*0.0508*0.0508; // [m^3]

//~~~~~ INITIALIZE CLASS AND FUNCTIONS ~~~~~//
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
//    GamepadMonitor* connectedGamepad; // pointer to Gamepad monitor
    double joystickVal;

    std::ofstream arduinoMotor0;
    std::ofstream arduinoMotor1;
    std::ofstream arduinoMotor2;
    std::ofstream arduinoMotor3;
    QSerialPort serial[4];


    bool controllerState;
    bool magneticPlotState;
    bool anglesPlotState;

    int currentControllerMode;
    bool enableAngleControlState;

    // System Variables
    double mAct[4] = {M*vAct, M*vAct, M*vAct, M*vAct}; //actuator magnet dipole moments (scalar) [Am^2] constant
    double pTool[3] = {0, 0, 0}; // tool position centered [m]
    double init_Angles[4] = {1.57079632679, 1.57079632679, 1.57079632679, 1.57079632679}; // Equilibrium for the unpowered system (pi/2)
    double next_Angles[4] = {0.0};
    double joystickValues[4] = {0.0, 0.0, 0.0, 0.0}; // stores the joystick values as the computer only reads changes in joystick position overtime
    // Optimization Variables
    double K[3][3] = {0.0};
    // double K[8][8] = {0.0}; // For full 8 magnet system
    double pAct[3][4], RzyAct[3][2][4], axesAct[3][4];

    double angle1 = 0.0; // Theta or Gamma
    double angle2 = 0.0; // Phi or Beta
    double B_mag = 0.0;
    // k factor is ( stiffness k / magnetic moment )
    double k_factor = 0.0; // mT/rad : use 7.4-8.1 from experiments
    double angle3 = 0.0; // N/A or theta


    int roll_dir = 0;

    double B_Desired[3] = {0.0};
    double B_Global_Desired[3] = {0.0};
//    double B_Global_Desired_Input[8] = {0.0};
    double B_Global_Output[3] = {0.0};
    double tilt = 0.0;
    double roll = 0.0;

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    // PUBLIC GENERAL MATHEMATICAL FUNCTIONS

    void MatrixMultVect(double C[3][3], double D[3], double E[3]); // DONE
    void MatrixMult3(double C[3][3], double D[3][3], double E[3][3]); // DONE
    void MatrixMult4(double C[4][4], double D[4][4], double E[4][4]); // DONE
    double norm(double A[12]); // DONE
    void MatrixInvert(double H[12][12], double result[12][12]); // DONE

    // PUBLIC FUNCTIONS WRITING TO MOTOR

    void ArduinoWrite(int motorNum, int position);
    void ArduinoWrite4(double motorAngles[4]);
    void updateAngles(void);


    // PUBLIC OPTIMIZATION FUNCTIONS
    void calculateTheta(void);
    void findB_SurgeonSimulator(double B_desired_local[3]);

    void tip2global(double tool_tilt, double tool_roll, double B_tip[3], double B_global_loc[3]); // DONE
    void find_Field(double pAct[3][4], double mAct[4], double RzyAct[3][2][4], double angles[4], double Bfield[3]); // DONE

    void f2_grad_hess(double pAct[3][4],  double RzyAct[3][2][4], double mAct[4], double pTool[3], double BGdes[8], double K[8][8], double arg0[12], double grad[12], double H[12][12]);
    void f2_grad_hess_3(double pAct[3][4],  double RzyAct[3][2][4], double mAct[4], double pTool[3], double BGdes[3], double K[3][3], double arg0[12], double grad[12], double H[12][12]);
    void find_angles(double pAct[3][4], double RzyAct[3][2][4], double mAct[4], double pTool[3], double BGdes[8], double K[8][8], double arg0[4], double angleAct0[4]);
    void find_angles_3(double pAct[3][4], double RzyAct[3][2][4], double mAct[4], double pTool[3], double BGdes[3], double K[3][3], double arg0[4], double angleAct0[4]);




private:
    Ui::MainWindow *ui;

    QVector<double> qv_t_mag, qv_Bx_mag, qv_By_mag, qv_Bz_mag, qv_t_ang, qv_motor0, qv_motor1, qv_motor2, qv_motor3;
    double currentTime = 0.0;
    // PRIVATE SETUP FUNCTIONS
    void setupPlots(void);
    void addPoints(double x_MagneticField, double y_MagneticField[3], double x_MotorAngles, double y_MotorAngles[4] );
    void plot(void);
    void setup_parameters(double pAct_sph[3][4], double rAct_sph[2][4], double pAct[3][4], double RzyAct[3][2][4], double axesAct[3][4] ); // DONE
    void init_K_3(double K[3][3], double maxB); // DONE 
    std::tuple<double, double, double> max_field_force_gradient( double pAct[3][4],double mAct[4], double mTool_mag ); // DONE
    void init_K(double K[8][8], double maxB, double maxG);

public slots:
    void enableController(void);
    void enableAngleControl(void);
    void enableMagneticPlot(void);
    void enableAnglesPlot(void);
    void ToolIN_press_Slot(void);
    void ToolOUT_press_Slot(void);
    void ToolINOUT_rel_Slot(void);
    void ToolINOUT_speed_Slot(int speed);
    void changeRoll(int dir);


private slots:
    void updateCaption(void);
    void updateMotors(void);
    void controllerModeSlot(int nModeIndex); // DONE
    void setTiltSlot(void);
    void setRollSlot(void);
    void setFactorSlot(void);
    void ResetMotorSlot(void);
    void clearTimeSlot(void);

signals:
    void controllerStateSignal(bool state);
};

#endif // MAINWINDOW_H
