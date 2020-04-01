#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "gamepadmonitor.h"
#include "qcustomplot.h"
#include <QtGamepad/QGamepad>
#include <QtSerialPort/QSerialPort>
#include <QDebug>
#include <QTimer>
#include <fstream>
#include <math.h>
#include <tuple>

namespace Ui {
class MainWindow;
}

//~~~~~ INITIALIZE MAGNETIC SYSTEM PARAMETERS ~~~~~//
// Constant regardless of setup
const int EncoderStepConversion = 3000; // Number of encoder steps per revolution
const int EncoderStepConversionTool = 360; //12000; // Number of encoder steps per revolution
// Changes if setup changes or based on tuning/calibration
const double r_mag = 0.156; //0.1259; // [m]
//const double minAllowableGrad = 0.001; // minimum allowable gradient of field
double pAct_sph[3][4] = { // actuator magnet positions in spherical coordinates (alpha and phi)
             { r_mag,    r_mag,    r_mag,    r_mag},   // Radius [m]
             {     0, 1*M_PI/2, 2*M_PI/2, 3*M_PI/2},   // Azimuth [deg]     alpha
             {M_PI/3,   M_PI/3,   M_PI/3,   M_PI/3}    // Inclination [deg] phi (polar?)
             };
double rAct_sph[2][4] = { // actuator magnet rotational axes in spherical coordinates (beta and gamma)
             {       0, 1*M_PI/2, 2*M_PI/2, 3*M_PI/2},    // Azimuth [deg]      beta
             {5*M_PI/6, 5*M_PI/6, 5*M_PI/6, 5*M_PI/6},    // Inclination [deg]  gamma
             };
// Untethered magnetic tool parameters
double mTool_dir[3] = {0, 0, -1}; // tool orientation
const double mTool_mag = 8.4e-3;  // tool dipole moment magnetiude [Am^2]
double maxB, maxF, maxG;
const int nAct = 4;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CONSTRUCTOR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setupPlots();

    // Connect Signals and Slots
    connect(ui->checkBox_controllerEnable,SIGNAL(clicked()),SLOT(enableController()));
    connect(ui->checkBox_angleControlSurgeon,SIGNAL(clicked()),SLOT(enableAngleControl()));
    connect(ui->comboBox_controllerMode,SIGNAL(currentIndexChanged (int)),SLOT( controllerModeSlot(int)) );
    connect(ui->doubleSpinBox_tilt,SIGNAL(valueChanged(double)),SLOT( setTiltSlot() ) );
    connect(ui->doubleSpinBox_roll,SIGNAL(editingFinished()),SLOT( setRollSlot() ) ); //valueChanged(double)
    connect(ui->doubleSpinBox_factor,SIGNAL(valueChanged(double)),SLOT( setFactorSlot() ) );
    connect(ui->checkBox_enableMagneticPlot,SIGNAL(clicked()),SLOT(enableMagneticPlot()));
    connect(ui->checkBox_enableAnglesPlot,SIGNAL(clicked()),SLOT(enableAnglesPlot()));

    connect(ui->pushButton_resetMotors,SIGNAL(clicked()),SLOT(ResetMotorSlot()));
    connect(ui->pushButton_clearTime,SIGNAL(clicked()),SLOT(clearTimeSlot()));
    connect(ui->pushButton_ToolIN,SIGNAL(pressed()),SLOT(ToolIN_press_Slot()));
    connect(ui->pushButton_ToolIN,SIGNAL(released()),SLOT(ToolINOUT_rel_Slot()));
    connect(ui->pushButton_ToolOUT,SIGNAL(pressed()),SLOT(ToolOUT_press_Slot()));
    connect(ui->pushButton_ToolOUT,SIGNAL(released()),SLOT(ToolINOUT_rel_Slot()));

    QTimer *captiontimer = new QTimer(this);
    connect(captiontimer, SIGNAL(timeout()), this, SLOT(updateCaption()));	// show fps,... in timer
    captiontimer->start(20); //Default 200. 20ms means 50Hz Refresh

//    QTimer *motortimer = new QTimer(this);
//    connect(motortimer, SIGNAL(timeout()), this, SLOT(updateMotors()));
//    motortimer->start(50); //Default 200. 20ms means 50Hz Refresh

    // Initialize Variables
    controllerState = ui->checkBox_controllerEnable->checkState();
    magneticPlotState = ui->checkBox_enableMagneticPlot->checkState();
    anglesPlotState = ui->checkBox_enableAnglesPlot->checkState();
    enableAngleControlState = ui->checkBox_angleControlSurgeon->checkState();
    currentControllerMode = ui->comboBox_controllerMode->currentIndex();

    // Initialize Magnetic Properties:
    setup_parameters(pAct_sph, rAct_sph, pAct, RzyAct, axesAct);
    std::tie(maxB, maxF, maxG) = max_field_force_gradient(pAct, mAct, mTool_mag);
    // init_K(K, maxB, maxG); // For full 8 magnet system
    init_K_3(K, maxB);
    find_Field(pAct, mAct, RzyAct, init_Angles, B_Global_Output);
//    for (int n=3;n<8;n++) // For full 8 magnet system
//    {
//        B_Global_Desired_Input[n] = minAllowableGrad;
//    }

    // Initialize from GUI
    tilt = ui->doubleSpinBox_tilt->value();
    roll = ui->doubleSpinBox_roll->value();
    k_factor = ui->doubleSpinBox_factor->value() / 1000;
    // set default labels to subwindow based on initial control mode
    controllerModeSlot(0);

    // Setup labels
    updateCaption();

    // Connect to motors
    /*
    Mapping b/w Ubuntu and Windows:
    Ubuntu: writeport = "/dev/ttyACM"
    Windows: writeport = "COM"
    Ubuntu      |   Windows
    0           |   6
    1           |   3
    2           |   4
    3           |   5
    4 (Trocar)  |   7
                |   8
    */
    std::string writeport = "COM";
    qDebug() << "Found the arduino port...\n";
    serial[0].setPortName(QString::fromStdString(writeport)+QString::number(6));
    serial[0].setBaudRate(QSerialPort::Baud115200);
    serial[0].setDataBits(QSerialPort::Data8);
    serial[0].setParity(QSerialPort::NoParity);
    serial[0].setStopBits(QSerialPort::OneStop);
    serial[0].setFlowControl(QSerialPort::NoFlowControl);
    serial[0].open(QIODevice::ReadWrite);
    for (int i=1;i<4;i++)
    {
            qDebug() << "Found the arduino port...\n";
            serial[i].setPortName(QString::fromStdString(writeport)+QString::number(i+2));
            serial[i].setBaudRate(QSerialPort::Baud115200);
            serial[i].setDataBits(QSerialPort::Data8);
            serial[i].setParity(QSerialPort::NoParity);
            serial[i].setStopBits(QSerialPort::OneStop);
            serial[i].setFlowControl(QSerialPort::NoFlowControl);
            serial[i].open(QIODevice::ReadWrite);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DESTROYER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
MainWindow::~MainWindow()
{
    arduinoMotor0.close();
    arduinoMotor1.close();
    arduinoMotor2.close();
    arduinoMotor3.close();
    delete ui;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MATH FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// ANDREWS MATH FUNCTIONS
void MainWindow::MatrixMultVect(double C[3][3], double D[3], double E[3])
{
    double num;
    for (int i=0; i<3; i++){
        num = 0.0;
        for(int k=0; k<3; k++){
          num = C[i][k]*D[k] + num;
        }
        E[i]=num;
    }
}

void MainWindow::MatrixMult4(double C[4][4], double D[4][4], double E[4][4])
{
    double num;
    for (int i=0; i<4; i++){
         for(int j=0; j<4; j++){
            num = 0.0;
            for(int k=0; k<4; k++){
              num = C[i][k]*D[k][j] + num;
            }
            E[i][j]=num;
         }
    }
}

void MainWindow::MatrixMult3(double C[3][3], double D[3][3], double E[3][3])
{
    double num;

    for (int i=0; i<3; i++){
         for(int j=0; j<3; j++){
            num = 0.0;
            for(int k=0; k<3; k++){
              num = C[i][k]*D[k][j] + num;
            }
            E[i][j]=num;
         }
    }
}

void MainWindow::tip2global(double tool_tilt, double tool_roll, double B_tip[3], double B_global_loc[3])
{
    double roty_tilt [3][3]= {{     cos(tool_tilt + M_PI), 0.0, sin(tool_tilt + M_PI)},
                              {	  			 	      0.0, 1.0,                   0.0},
                              {-1.0*sin(tool_tilt + M_PI), 0.0, cos(tool_tilt + M_PI)}};
    double rotz_roll [3][3]= {{cos(tool_roll), -1.0*sin(tool_roll), 0.0},
                              {sin(tool_roll),      cos(tool_roll), 0.0},
                              {           0.0,                 0.0, 1.0}};
    double rot_total[3][3];
    MatrixMult3(roty_tilt,rotz_roll,rot_total);
    /* 	Assume the origin of the tool frame is 0 with respect to the
            global frame - otherwise, further matrix transformation is required */
    MatrixMultVect(rot_total,B_tip,B_global_loc);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// ADAMS MATH FUNCTIONS
double MainWindow::norm(double A[12])
{
    double result = 0;
    for (int n=0; n<8; n++)
    {
        result += A[n]*A[n];
    }
    result = sqrt(result);
    return result;
}

void MainWindow::MatrixInvert(double H[12][12], double result[12][12])
{
  // This function finds the inverse of a 12 x 12 matrix using Guassian Elimination,
  // or Reduced-row Echelon form.
  // Create a 12 x 24 Matrix of the form [ H | I ] where I is the identity matrix
  double invert[12][24] = {0};
  for (int i=0; i<12; i++)
  {
    for (int j=0; j<12; j++)
    {
      invert[i][j] = H[i][j];
    }
    invert[i][i+12] = 1;
  }
  // Matrix now looks like [ H | I ] Matrix
  // Perform reduced row-echelon algorithm.
  for (int i=0; i<12-1; i++)
  {
    double factor = invert[i][i]; //diagonal value on left side
    // Normalize rows so leading value is 1
    for (int j=0; j<24; j++)
    {
      invert[i][j] = invert[i][j]/factor;
    }
    // leading row should be 1 now
    // Subtract row from lower rows
    for (int j=i+1; j<12; j++)
    {
      factor = invert[j][i]; //value below the leading 1
      for (int k=0; k<24; k++)
      {
        invert[j][k] = invert[j][k] - invert[i][k]*factor; // top row above multiplied by correction factor subtracted from current row
      }
    }

  }
  // Normalizing last row
  double factor = invert[11][11]; //diagonal value
  // Normalize final row
    // if( abs(factor) > 1e-16 )
    // {
        for (int j=0; j<24; j++)
    {
        invert[11][j] = invert[11][j]/factor;
    }
    // }
  // Should look like:
  // {1 a b c d | k l m n o}
  // {0 1 e f g | p q r s t}
  // {0 0 1 h i | u v w x y}
  // {0 0 0 1 j | z .......}
  // {0 0 0 0 1 | .........}
  // Now work upwards,
  for (int i=12-1; i>=0+1; i--)
  {
    for (int j=i-1; j>=0; j--)
    {
      factor = invert[j][i]; // value above diagonal
      for (int k=0; k<24; k++)
      {
        invert[j][k] = invert[j][k] - invert[i][k]*factor; // bottom row below multiplied by correction factor subtracted from current row
      }
    }
  }
  // record result
  for (int i = 0; i < 12; i++)
  {
      for (int j = 0; j < 12; j++)
      {
        result[i][j] = invert[i][j+12];
      }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::setupPlots(void)
{
  // The following plot setup is mostly taken from the plot demos:
  ui->plot_magnetic->addGraph();
  ui->plot_magnetic->graph()->setPen(QPen(QColor(189, 19, 62, 255))); // U of T Highlight Red  //QPen(Qt::red));
  ui->plot_magnetic->addGraph();
  ui->plot_magnetic->graph()->setPen(QPen(QColor(34, 139, 34, 255))); // Forest Green          //Qt::green));
  ui->plot_magnetic->addGraph();
  ui->plot_magnetic->graph()->setPen(QPen(QColor(6, 41, 88, 255))); // U of T Blue             //Qt::blue));
  ui->plot_magnetic->xAxis->setLabel("Time [s]");
  ui->plot_magnetic->yAxis->setLabel("Magnetic Field [mT]");
  ui->plot_magnetic->yAxis->setRange(-20, 20);
  // Next one
  ui->plot_angles->addGraph();
  ui->plot_angles->graph()->setPen(QPen(QColor(6, 41, 88, 255))); // U of T Blue
  ui->plot_angles->addGraph();
  ui->plot_angles->graph()->setPen(QPen(QColor(34, 139, 34, 255))); // Forest Green
  ui->plot_angles->addGraph();
  ui->plot_angles->graph()->setPen(QPen(QColor(255, 118, 25, 255))); // Pumpkin Orange
  ui->plot_angles->addGraph();
  ui->plot_angles->graph()->setPen(QPen(QColor(189, 19, 62, 255))); // U of T Highlight Red
  ui->plot_angles->xAxis->setLabel("Time [s]");
  ui->plot_angles->yAxis->setLabel("Angle [rad]");
  ui->plot_angles->yAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
  ui->plot_angles->yAxis->setRange(0, 2*M_PI);
    // Able to change the view with the mouse
  ui->plot_magnetic->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui->plot_angles->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

void MainWindow::setup_parameters(double pAct_sph[3][4], double rAct_sph[2][4], double pAct[3][4], double RzyAct[3][2][4], double axesAct[3][4] )
{
    // function [pAct,RzyAct,axesAct] = setup_parameters(pAct_sph, rAct_sph)
    // % Inputs:
    // % pAct_sph = actuator magnet positions in spherical coordinates [radius(m); azimuth(deg); inclination (deg)];
    // % rAct_sph = actuator magnet rotational axes in spherical coordinates [azimuth(deg); inclination (deg)];
    // % Outputs:
    // % pAct: xyz coordinates of actuator magnet centers [m]
    // % RzyAct: ZY Euler angle rotation matrix for each actuator magnet
    // % axesAct: direction of axis of rotation in xyz

    // % Author: Patrick Ryan
    // % Adapted by: Adam Schonewille
    // % Rewritten in C++ from MATLAB by Adam Schonewille

//    int nAct = 4; // number of magnets
    // Magnet locations
    for (int n=0; n<nAct; n++)
    {
        pAct[0][n] = pAct_sph[0][n]*sin(pAct_sph[2][n])*cos(pAct_sph[1][n]);
        pAct[1][n] = pAct_sph[0][n]*sin(pAct_sph[2][n])*sin(pAct_sph[1][n]);
        pAct[2][n] = pAct_sph[0][n]*cos(pAct_sph[2][n]);
    }
    // Axes & constants
    // Omega of revolution
    for (int n=0; n<nAct; n++)
    {
      axesAct[0][n] = sin(rAct_sph[1][n])*cos(rAct_sph[0][n]);
      axesAct[1][n] = sin(rAct_sph[1][n])*sin(rAct_sph[0][n]);
      axesAct[2][n] = cos(rAct_sph[1][n]);
    }
    for (int i=0; i<nAct; i++)
    {
        RzyAct[0][0][i] =      cos(rAct_sph[1][i])*cos(rAct_sph[0][i]);
        RzyAct[0][1][i] = -1.0*sin(rAct_sph[0][i]);
        RzyAct[1][0][i] =      cos(rAct_sph[1][i])*sin(rAct_sph[0][i]);
        RzyAct[1][1][i] =      cos(rAct_sph[0][i]);
        RzyAct[2][0][i] = -1.0*sin(rAct_sph[1][i]);
        RzyAct[2][1][i] =  0.0;
    }
}

std::tuple<double, double, double> MainWindow::max_field_force_gradient( double pAct[3][4],double mAct[4], double mTool_mag )
{
    //function [ maxB, maxF, maxG ] = max_field_force_gradient( pAct, mAct, mTool_mag )
    // % Determine the maximum field, force, and field gradient magnitude that can be produced
    // % if all the magnetic volume is concentrated at a single point at R dist from the workspace
    // % These values are useful for scaling the relative field, force, and gradient terms

//    int nAct = 4;

    double maxB = 0;
    double maxF = 0;
    double maxG = 0;

    for (int i=0; i<nAct; i++)
    {
        // All alligned in one direction ie m is parallel to r
        maxB = maxB + mu_0 * mAct[i] / ( 2 * M_PI * pow( pAct[0][i]*pAct[0][i]+pAct[1][i]*pAct[1][i]+pAct[2][i]*pAct[2][i], (1.5) ) );

        // This is maximum magnitude of a single element of 3x3 matrix
        // m is parallel to r but opposite dir.
        maxG = maxG + 3 * mu_0 * mAct[i] / ( 2 * M_PI * pow( pAct[0][i]*pAct[0][i]+pAct[1][i]*pAct[1][i]+pAct[2][i]*pAct[2][i] , (2)) );

        maxF = maxF + 3 * mu_0 * mAct[i] / ( 2 * M_PI * pow( pAct[0][i]*pAct[0][i]+pAct[1][i]*pAct[1][i]+pAct[2][i]*pAct[2][i] , (2) ) ) * mTool_mag;
    }
    return std::make_tuple(maxB, maxF, maxG);
}

void MainWindow::init_K_3(double K[3][3], double maxB)
{
    for (int i = 0; i < 3; i++)
    {
        K[i][i] = 1/maxB;
    }
}

void MainWindow::find_Field(double pAct[3][4], double mAct[4], double RzyAct[3][2][4], double angles[4], double Bfield[3])
{
    // GET Bpr AGAIN
    double Bpr[3][8] = {0.0};
    for (int i=0; i<nAct; i++)
    {
            // Position Vectors
            double p[3] = { -pAct[0][i]+pTool[0], -pAct[1][i]+pTool[1], -pAct[2][i]+pTool[2] };
            double p_norm = sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
            double p_hat[3] = {p[0]/p_norm, p[1]/p_norm, p[2]/p_norm };
            // Field and Gradient Equation constants
            double KB =   mAct[i]*1e-7 / (p_norm*p_norm*p_norm); // p_norm^3
            // Find Bpr
            int eye3[3][3] = { {1,0,0},
                               {0,1,0},
                               {0,0,1} };
            double p_temp_calc[3][3];
            for (int j=0; j<3; j++)
            {
                    double accum_0 = 0;
                    double accum_1 = 0;
                    for (int k=0; k<3; k++)
                    {
                            p_temp_calc[j][k] = 3*p_hat[j]*p_hat[k] - eye3[j][k];
                            accum_0 = accum_0 + KB*p_temp_calc[j][k]*RzyAct[k][0][i];
                            accum_1 = accum_1 + KB*p_temp_calc[j][k]*RzyAct[k][1][i];
                    }
                    // 3 by 2 done 4 times to get 3 by 8
                    Bpr[j][2*i+0] = accum_0;
                    Bpr[j][2*i+1] = accum_1;
            }
        }
        // Actuator Vector:
        double anglesVect[8] = {0.0};
        for (int k=0; k<4; k++)
        {
            anglesVect[2*k] = cos(angles[k]);
            anglesVect[2*k+1] = sin(angles[k]);
        }
        // Calc B-field using Bpr and anglesVect
        for (int i=0; i<3; i++)
        {
            double accum = 0.0;
            for (int j=0; j<8; j++)
            {
                accum = accum + anglesVect[j]*Bpr[i][j];
            }
            Bfield[i] = accum;
        }

}

void MainWindow::calculateTheta()
{
    // For Surgeon Simulator control mode the angles are as follows:
    // angle3 : Theta, the angle between the z-axis and the desired heading of the gripper
    // angle1 : Gamma, the x plane rotation angle
    // angle2 : Beta, the out of plane rotation angle
//    double test = atan( sqrt( pow(sin(angle2),2)+pow(cos(angle2),2)*pow(sin(angle1),2) ) / (cos(angle1)*cos(angle2)) );
//    qDebug() << test;

    angle3 = atan( sqrt( pow(sin(angle2),2)+pow(cos(angle2),2)*pow(sin(angle1),2) ) / (cos(angle1)*cos(angle2)) );
}

void MainWindow::findB_SurgeonSimulator(double B_desired_local[3])
{
    // closing the gripper requires a B field parallel to the (open-loop) direction of the gripper
    double B_parallel[3] = { B_mag*cos(angle2)*sin(angle1), B_mag*pow(sin(angle2),2), B_mag*cos(angle2)*cos(angle1)};
    double det = sqrt( pow(cos(angle2),2)*pow(sin(angle1),2) + pow(sin(angle2),2) );
    double constantInFront = k_factor*angle3/det;
    double B_perp[3] = {constantInFront*pow(cos(angle2),2)*sin(angle1)*cos(angle1), constantInFront*sin(angle2)*cos(angle1)*cos(angle2), -constantInFront*(pow(cos(angle2),2)*pow(sin(angle1),2) + pow(sin(angle2),2)) };

    // Check that there was not a divide by zero error from using det=0 when angle1=0 and angle2=0
    if ( isnan(B_perp[0]) || isnan(B_perp[1]) || isnan(B_perp[2]) )
    {
        for (int i=0;i<3;i++)
        {
            B_desired_local[i] = B_parallel[i]; // Ignore B_perp as it should be 0 from the calculation of 0/0;
        }
    }
    else
    {
        // Calculate the desired Local B-field from parallel and perpendicular components
        for (int i=0;i<3;i++)
        {
            B_desired_local[i] = B_parallel[i] + B_perp[i];
        }
    }
}

//~~~~~~~~~~~~~~~~~~ PLOTTING FUNCTIONS ~~~~~~~~~~~~~~~~~//

void MainWindow::addPoints(double x_MagneticField, double y_MagneticField[], double x_MotorAngles, double y_MotorAngles[])
{
//    qDebug() << y_MagneticField[0];
    qv_t_mag.append(x_MagneticField);
    qv_Bx_mag.append(y_MagneticField[0]*1000);
    qv_By_mag.append(y_MagneticField[1]*1000);
    qv_Bz_mag.append(y_MagneticField[2]*1000);
    qv_t_ang.append(x_MotorAngles);
    double wrappedAngles[4] = {0.0};
    for (int i=0;i<4;i++)
    {
        wrappedAngles[i] = fmod(y_MotorAngles[i], 2*M_PI);
        if (wrappedAngles[i] < 0)
        {
            wrappedAngles[i] = wrappedAngles[i] + 2*M_PI;
        }
    }
    qv_motor0.append(wrappedAngles[0]);
    qv_motor1.append(wrappedAngles[1]);
    qv_motor2.append(wrappedAngles[2]);
    qv_motor3.append(wrappedAngles[3]);
}

void MainWindow::plot()
{
    if (magneticPlotState)
    {
        ui->plot_magnetic->graph(0)->setData(qv_t_mag, qv_Bx_mag);
        ui->plot_magnetic->graph(1)->setData(qv_t_mag, qv_By_mag);
        ui->plot_magnetic->graph(2)->setData(qv_t_mag, qv_Bz_mag);
        ui->plot_magnetic->xAxis->setRange(currentTime-10, currentTime);
        ui->plot_magnetic->replot();
        ui->plot_magnetic->update();
    }
    if (anglesPlotState)
    {
        ui->plot_angles->graph(0)->setData(qv_t_ang, qv_motor0);
        ui->plot_angles->graph(1)->setData(qv_t_ang, qv_motor1);
        ui->plot_angles->graph(2)->setData(qv_t_ang, qv_motor2);
        ui->plot_angles->graph(3)->setData(qv_t_ang, qv_motor3);
        ui->plot_angles->xAxis->setRange(currentTime-10, currentTime);
        ui->plot_angles->replot();
        ui->plot_angles->update();
    }
}


//~~~~~~~~~~~~~~~ OPTIMIZATION FUNCTIONS ~~~~~~~~~~~~~~~//

void MainWindow::find_angles_3(double pAct[3][4], double RzyAct[3][2][4], double mAct[4], double pTool[3], double BGdes[3], double K[3][3], double arg0[4], double angleAct0[4])
{
  // function [angleAct0, reachable] = find_angles(pAct, RzyAct, mAct, pTool, BGdes, K, arg0)
  // % Inputs:
  // % pAct: xyz coordinates of actuator magnet centers (m)
  // % RzyAct: ZY Euler angle rotation matrix for each actuator magnet
  // % mAct: magnetic dipole moment of actuator magnets [Am^2]
  // % angleAct: angular position of magnets [rad]
  // % pTool: xyz coordinates of tool in workplace [m]
    // % see:
    // https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization
    // int nAct = 4; // number of actuator magnets - not used??
    // double lambda0[4]; //  n - not used??
    // double phi0[8];    // 2n - not used??

    // Before the code checked to see if the input was already filled without
    // phi/lambda values but this code assumes that the argument is the Initial
    // starting angles of the motors
    double argument0[12] = {cos(arg0[0]), sin(arg0[0]), cos(arg0[1]), sin(arg0[1]), cos(arg0[2]), sin(arg0[2]), cos(arg0[3]), sin(arg0[3]), 0.0, 0.0, 0.0, 0.0 }; // arg0 = angleAct0

    // this probably only needs to be > 1e-6 in magnitude
    double grad[12] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
    int count = 0;
    double argument1[12] = {0.0};

    while ( norm(grad) > 1e-6 && count < 10)
    {
        double H[12][12] = {0.0}; // 3n by 3n
        // AP's default parameters
        f2_grad_hess_3(pAct, RzyAct, mAct, pTool, BGdes, K, argument0, grad, H);   // TODO:
        // update optimziation variables

                double step = 0.3;
                for (int k=0;k<12;k++)
                {
                    H[k][k] = H[k][k]+step;
                }

                // cout << "Hessian:  " << endl;
            // for (int p=0; p<12; p++)
            // {
                // 	for (int q=0; q<12; q++)
                // 	{
                // 		cout << H[p][q] << "   ";
                // 	}
                // 	cout << endl;
            // }
            // cout << endl;

        double H_inv[12][12] = {0.0};
        MatrixInvert(H, H_inv);

                // cout << "Hessian Invert:  " << endl;
            // for (int p=0; p<12; p++)
            // {
                // 	for (int q=0; q<12; q++)
                // 	{
                // 		cout << H_inv[p][q] << "   ";
                // 	}
                // 	cout << endl;
            // }
            // cout << endl;


        for ( int i=0; i<12; i++ )
        {
                double accum = 0;
                for ( int k=0; k<12; k++)
            {
                    accum = accum + H_inv[i][k]*grad[k];
                }
                argument1[i] = argument0[i]-0.5*accum;
                        // argument1[i] = argument0[i]-accum;
              argument0[i] = argument1[i];
        }
        count = count+1;
    }

    for (int m=0; m<4; m++)
    {
        // sin(angles) and cos(angles) are the first 8 entries of argument0
        // These are also refered to as the phi0's
        angleAct0[m] = atan2( argument0[2*m+1],argument0[2*m] );
                while (arg0[m] - angleAct0[m] > M_PI )
                    angleAct0[m] = angleAct0[m] + 2*M_PI;
                while (arg0[m] - angleAct0[m] < -M_PI )
                    angleAct0[m] = angleAct0[m] - 2*M_PI;
    }
    //reachable = norm(lambda0) < 1e-6; // boolean describing the outcome of the algorithm
}

void MainWindow::f2_grad_hess_3(double pAct[3][4],  double RzyAct[3][2][4], double mAct[4], double pTool[3], double BGdes[3], double K[3][3], double arg0[12], double grad[12], double H[12][12])
{
    // function [f2,grad,H] = f2_grad_hess(pAct, RzyAct, mAct, pTool, BGdes, K, arg0)
    // % This function returns the function value, gradient, and hessian of the constrained quadratic optimization problem as forumlated
    // % by Andrew Petruska. This formulation converts the error between a desired field/gradient and actual field/gradient from non-linear
    // % values of theta to linear values of phi with non-linear lagrangian terms (lambdas).

    // % Inputs:
    // % pAct: xyz coordinates of actuator magnet centers (m)
    // % RzyAct: ZY Euler angle rotation matrix for each actuator magnet
    // % mAct: magnetic dipole moment of actuator magnets [Am^2]
    // % angleAct: angular position of magnets [rad]
    // % pTool: xyz coordinates of tool in workplace [m]
    // % BGdes: 8x1 vector containing desired field (3x1) and desired gradient (in 5x1 packing)
    // % K: a diagonal matrix used to weigh the field and gradient terms (to account for different units)
    // % arg0: initial guess for search

    // % Outputs:
    // % f2: the function value at arg0 (error calculation not included in the c++ version)
    // % grad: the function gradient at arg0 (derivative of function value with arg0)
    // % hess: the function hessian at arg0 (2nd derivatibe of function value with arg0)

//    int nAct = 4;
    double lambda0[4]; //  n
    double phi0[8];    // 2n

    for (int j=0; j<4; j++)
    {
      lambda0[j] = arg0[j+8]; // assign lambda
      phi0[j] = arg0[j];      // assign the first 4 of phi
    }
    for (int j=0; j<4; j++)
    {
      phi0[j+4] = arg0[j+4];  // assign the last 4 of phi
    }
    // Calculate BG matrix %! -this matrix is fixed, can calculate outside function to save time
    // Adam did not write this comment ^ and does not understand the implications
    double Bpr[3][8] = {0.0}; // zeros(3,nAct*2);
    // //double Gpr[5][8] = {0.0}; // zeros(5,nAct*2);

    for (int i=0; i<nAct; i++)
    {
        // Position Vectors
        double p[3] = { -pAct[0][i]+pTool[0], -pAct[1][i]+pTool[1], -pAct[2][i]+pTool[2] };
        double p_norm = sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
        double p_hat[3] = {p[0]/p_norm, p[1]/p_norm, p[2]/p_norm };
        // Field and Gradient Equation constants
        double KB =   mAct[i]*1e-7 / (p_norm*p_norm*p_norm); // p_norm^3
        // //double KG = 3*mAct[i]*1e-7 / (p_norm*p_norm*p_norm*p_norm); // p_norm^4
        // Find Bpr
        int eye3[3][3] = { {1,0,0},
                           {0,1,0},
                           {0,0,1} };
        double p_temp_calc[3][3];
        for (int j=0; j<3; j++)
        {
            double accum_0 = 0;
            double accum_1 = 0;
            for (int k=0; k<3; k++)
            {
                p_temp_calc[j][k] = 3*p_hat[j]*p_hat[k] - eye3[j][k];
                accum_0 = accum_0 + KB*p_temp_calc[j][k]*RzyAct[k][0][i];
                accum_1 = accum_1 + KB*p_temp_calc[j][k]*RzyAct[k][1][i];
            }
            // 3 by 2 done 4 times to get 3 by 8
            Bpr[j][2*i+0] = accum_0;
            Bpr[j][2*i+1] = accum_1;
        }
        // 5 by 3
        // double Gp[5][3] = {
        //     { 3*p_hat[0]-5*pow(p_hat[0],3),              p_hat[1]-5*pow(p_hat[0],2)*p_hat[1],     p_hat[2]-5*pow(p_hat[0],2)*p_hat[2]  },
        //     {   p_hat[1]-5*pow(p_hat[0],2)*p_hat[1],     p_hat[0]-5*p_hat[0]*pow(p_hat[1],2),  -5*p_hat[0]*p_hat[1]*p_hat[2]           },
        //     {   p_hat[2]-5*pow(p_hat[0],2)*p_hat[2],  -5*p_hat[0]*p_hat[1]*p_hat[2],              p_hat[0]-5*p_hat[0]*pow(p_hat[2],2)  },
        //     {   p_hat[0]-5*p_hat[0]*pow(p_hat[1],2),   3*p_hat[1]-5*pow(p_hat[1],3),              p_hat[2]-5*pow(p_hat[1],2)*p_hat[2]  },
        //     {-5*p_hat[0]*p_hat[1]*p_hat[2],              p_hat[2]-5*pow(p_hat[1],2)*p_hat[2],     p_hat[1]-5*p_hat[1]*pow(p_hat[2],2)  }
        //     };

        // for (int j=0; j<5; j++)
        // {
        //     double accum_0 = 0;
        //     double accum_1 = 0;
        //     for (int k=0; k<3; k++)
        //     {
        //         accum_0 = accum_0 + KG*Gp[j][k]*RzyAct[k][0][i];
        //         accum_1 = accum_1 + KG*Gp[j][k]*RzyAct[k][1][i];
        //     }
        //     // 3 by 2 done 4 times to get 3 by 8
        //     Gpr[j][2*i+0] = accum_0;
        //     Gpr[j][2*i+1] = accum_1;
        // }
        // 5 by 8
    }
    // 8 by 8
    // BG = [Bpr; Gpr]; // Bpr on top of Gpr

        // BG is now just Bpr a 3 x 8 matrix

        double BG[3][8] = {0};
    for (int j=0; j<8; j++)
    {
        for (int k=0; k<3; k++)
        {
            BG[k][j] = Bpr[k][j];
        }
        // for (int k=0; k<5; k++)
        // {
        //     BG[k+3][j] = Gpr[k][j];
        // }
    }
    // W is Scalar and the inverted trace of some stuff
    // W = 1 / trace( BG'* K.^2 * BG); //!'
    double W = 0;
    for (int j=0; j<8; j++)
    {
        for (int k=0; k<3; k++)
        {
            W = W + K[k][k] * K[k][k] * BG[k][j] * BG[k][j];
        }
    }
    W = 1.0/W;

    //~~~ GRADIENT CALCULATION ~~~//
    // 12 BY 1 VECTOR
    double dgr_dla[4] = {0.0};

    for (int i=0; i<nAct; i++)
    {
        dgr_dla[i] = (phi0[2*i]*phi0[2*i] + phi0[2*i+1]*phi0[2*i+1] - 1); // %!
    }

    // lambda diagonal with elements repeating twice in a row
    double la_matr[8][8] = {{lambda0[0],          0,          0,          0,          0,          0,          0,          0 },
                            {         0, lambda0[0],          0,          0,          0,          0,          0,          0 },
                            {         0,          0, lambda0[1],          0,          0,          0,          0,          0 },
                            {         0,          0,          0, lambda0[1],          0,          0,          0,          0 },
                            {         0,          0,          0,          0, lambda0[2],          0,          0,          0 },
                            {         0,          0,          0,          0,          0, lambda0[2],          0,          0 },
                            {         0,          0,          0,          0,          0,          0, lambda0[3],          0 },
                            {         0,          0,          0,          0,          0,          0,          0, lambda0[3] } };
    // Intermediate calculation
    double BBGG[3]= {0};
    for (int i=0; i<3; i++)
    {
        BBGG[i] = K[i][i]*K[i][i] * ( BG[i][0]*phi0[0]+BG[i][1]*phi0[1]+BG[i][2]*phi0[2]+BG[i][3]*phi0[3]+BG[i][4]*phi0[4]+BG[i][5]*phi0[5]+BG[i][6]*phi0[6]+BG[i][7]*phi0[7] - BGdes[i] );
    }
    // full calc
        // cout << "Gradient:  " << endl;

    for (int j=0; j<8; j++)
    {
        double accum = 0;
        for (int k=0; k<3; k++)
        {
            accum = accum + BG[k][j]*BBGG[k];
        }
        grad[j] = 2*W*accum + 2*la_matr[j][j]*phi0[j]; // 2 comes from derivative terms
                // cout << grad[j] << endl;
    }
    // Last part of the gradient is just the dgr_dla terms
    for (int m=0; m<4; m++)
    {
        grad[m+8] = dgr_dla[m];
                // cout << grad[m+8] << endl;
    }

    //~~~ HESSIAN CALCULATION ~~~//
    // Clear  H to zero
    for (int x=0; x<12; x++)
    {
      for (int y=0; y<12; y++)
      {
        H[x][y] = 0.0;
      }
    }
    double dhe_dl[8][4] = {0.0}; //zeros(nAct*2,nAct);
    for (int i=0; i<nAct; i++)
    {
        dhe_dl[2*i][i]   = phi0[2*i];
        dhe_dl[2*i+1][i] = phi0[2*i+1];
    }
    // 12 by 12
    // H = [
    //     // factor of 2 larger than AP code
    //     2*W*(BG'*K.^2*BG) + 2*la_matr     2*dhe_dl;     // '

    //     2*dhe_dl'                    zeros(nAct);       // '
    //     ];

    // find BG'*K.^2*BG
      double tmpBG[8][8] = {0.0};
      for (int i=0; i<8; i++)
    {
            for (int j=0; j<8; j++)
        {
                   double accum = 0;
                   for (int k=0; k<3; k++)
             {
                        accum = accum + K[k][k]*K[k][k]*BG[k][i]*BG[k][j];
                   }
               tmpBG[i][j] = accum;
             }
      }
    // 12 by 12
    for (int j=0; j<8; j++)
    {
        for (int k=0; k<8; k++)
        {
                        H[j][k] = 2.0*W*tmpBG[j][k] + 2.0*la_matr[j][k];
        }
    }
    for (int n=0; n<4; n++)
    {
        H[2*n][n+8]   = 2.0*dhe_dl[2*n][n];
        H[2*n+1][n+8] = 2.0*dhe_dl[2*n+1][n];
        H[n+8][2*n]   = 2.0*dhe_dl[2*n][n];
        H[n+8][2*n+1] = 2.0*dhe_dl[2*n+1][n];
    }
}

//~~~~~~~~~~~~~~~~~~ ARDUINO FUNCTIONS ~~~~~~~~~~~~~~~~~//

//--------------------------------------------------------//
void MainWindow::ArduinoWrite(int motorNum, int position)
{
    std::string writeport = "COM";
    writeport = writeport + std::to_string(motorNum);

    std::string nVal = std::to_string( position );
    nVal = nVal + ',';
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
//    std::ofstream arduino;
//    arduino.open( writeport );
//    arduino << nVal;
//    arduino.close();
}
//------------------------------------------------------------------------
//
void MainWindow::ArduinoWrite4(double motorAngles[4])
{
    int position[4];
    // Takes 4 motor angles in radians and converts to encoder steps and sends
    // the correct command to the respective Arduinos
    for (int n=0; n<4; n++)
    {
        position[n] = (int) (motorAngles[n]*EncoderStepConversion/(2*M_PI));
        // cout << position << endl;
        position[n] = position[n] + EncoderStepConversion; // shift 1 revolution to avoid sending negative numbers
        // cout << position << endl;
//        ArduinoWrite(n, position);
        // Utilize other function to rid unecessary rewritten code
    }
    // Send to the motors
    QByteArray q_b[4];
    for (int i=0;i<4;i++)
        q_b[i].setNum(position[i]);
    serial[0].write(q_b[0]);
    serial[1].write(q_b[1]);
    serial[2].write(q_b[2]);
    serial[3].write(q_b[3]);
}

void MainWindow::updateAngles(void)
{
    // Overwrite previous angles with current ones
    for (int x=0;x<nAct;x++)
    {
        init_Angles[x] = next_Angles[x];
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~ SLOTS ~~~~~~~~~~~~~~~~~~~~~~//

void MainWindow::updateCaption(void)
{
    ui->label_status->setText(tr("%1").arg(joystickVal));
//    ui->label_test->setText((QString)controllerState);
    // Only round the output text, not the actual saved values
    ui->label_Bx_desired->setText(tr("%1").arg(round(B_Desired[0]*1000000)/1000));
    ui->label_By_desired->setText(tr("%1").arg(round(B_Desired[1]*1000000)/1000));
    ui->label_Bz_desired->setText(tr("%1").arg(round(B_Desired[2]*1000000)/1000));
    ui->label_Bx_desired_global->setText(tr("%1").arg(round(B_Global_Desired[0]*1000000)/1000));
    ui->label_By_desired_global->setText(tr("%1").arg(round(B_Global_Desired[1]*1000000)/1000));
    ui->label_Bz_desired_global->setText(tr("%1").arg(round(B_Global_Desired[2]*1000000)/1000));
    ui->label_Bx_output->setText(tr("%1").arg(round(B_Global_Output[0]*1000000)/1000));
    ui->label_By_output->setText(tr("%1").arg(round(B_Global_Output[1]*1000000)/1000));
    ui->label_Bz_output->setText(tr("%1").arg(round(B_Global_Output[2]*1000000)/1000));
    ui->label_motor0_deg->setText(tr("%1").arg(round(init_Angles[0]*180/M_PI*1000)/1000));
    ui->label_motor1_deg->setText(tr("%1").arg(round(init_Angles[1]*180/M_PI*1000)/1000));
    ui->label_motor2_deg->setText(tr("%1").arg(round(init_Angles[2]*180/M_PI*1000)/1000));
    ui->label_motor3_deg->setText(tr("%1").arg(round(init_Angles[3]*180/M_PI*1000)/1000));
    ui->label_angle1->setText(tr("%1").arg(round(angle1*180/M_PI*1000)/1000));
    ui->label_angle2->setText(tr("%1").arg(round(angle2*180/M_PI*1000)/1000));
    ui->label_B_mag->setText(tr("%1").arg(round(B_mag*1000000)/1000));

    // Update the Roll of the device
    changeRoll(roll_dir);
    currentTime = currentTime + 1/50.0;
//    double temppp[4] = {0.0};
    addPoints(currentTime, B_Global_Output, currentTime, init_Angles);
//    ui-> B_Global_Output
    plot();
}
void MainWindow::updateMotors(void)
{
    ArduinoWrite4(init_Angles);
}

void MainWindow::enableController(void)
{
    qDebug() << ui->checkBox_controllerEnable->checkState();
    controllerState = ui->checkBox_controllerEnable->checkState();
    emit controllerStateSignal(controllerState);

//    enableController = checkBox_PS3_Controller->checkState();
    if (!controllerState)
    {
        B_Desired[0] = 0;
        B_Desired[1] = 0;
        B_Desired[2] = 0;
        angle1 = 0;
        angle2 = 0;
        angle3 = 0;
        B_mag = 0;
    }
    tip2global(tilt, roll, B_Desired, B_Global_Desired);
    //double tool_tilt, double tool_roll, double B_tip[3], double B_global_loc[3]

}

void MainWindow::enableAngleControl(void)
{
    enableAngleControlState = ui->checkBox_angleControlSurgeon->checkState();
}

void MainWindow::enableMagneticPlot(void)
{
    magneticPlotState = ui->checkBox_enableMagneticPlot->checkState();
}

void MainWindow::enableAnglesPlot(void)
{
    anglesPlotState = ui->checkBox_enableAnglesPlot->checkState();
}

//------------------------------------------------------------------------
//
void MainWindow::ToolIN_press_Slot(void)
{
    std::string writeport = "COM7"; // Hardcoded as ACM4
    // writeport = writeport + std::to_string(motorNum);
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string message = "l1,";
    arduino << message;
    // cout << message;
    arduino.close();
}
//------------------------------------------------------------------------
//
void MainWindow::ToolINOUT_rel_Slot(void)
{
    std::string writeport = "COM7";
    // writeport = writeport + std::to_string(motorNum);
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string message = "l0,";
    arduino << message;
    // cout << message;
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
    arduino.close();
}
//------------------------------------------------------------------------
//
void MainWindow::ToolINOUT_speed_Slot(int speed)
{
    std::string writeport = "COM7";
    // writeport = writeport + std::to_string(motorNum);
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    qDebug() << speed;
    std::string message = std::to_string(speed); //10-210
    message = "l" + message + ",";
    arduino << message;
    // cout << message;
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
    arduino.close();
}
//------------------------------------------------------------------------
//
void MainWindow::ToolOUT_press_Slot(void)
{
    std::string writeport = "COM7";
    // writeport = writeport + std::to_string(motorNum);
    // Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string message = "l2,";
    arduino << message;
    // cout << message;
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
    arduino.close();
}
//------------------------------------------------------------------------
//
void MainWindow::controllerModeSlot(int nModeIndex)
{
    // Only necessary component for function is to record the current mode number
    currentControllerMode = nModeIndex;

    // Change GUI variables to show the user relevant information
    if (currentControllerMode == 2)
    {
        ui->labelname_Type_of_Control->setText("Polar Field Control Mode");
        ui->labelname_Angle1->setText("θ : " );
        ui->labelname_Angle2->setText("ϕ : ");
        ui->labelname_B_magnitude->setText("B mag : ");
        ui->labelname_factor->setText("N/A");

    }
    else if (currentControllerMode == 3)
    {
        ui->labelname_Type_of_Control->setText("Surgeon Simulator Control Mode");
        ui->labelname_Angle1->setText("γ : "); // Gamma
        ui->labelname_Angle2->setText("β : "); // Beta
        ui->labelname_B_magnitude->setText("B parallel : ");
        ui->labelname_factor->setText("k factor : ");
    }
    else
    {
        ui->labelname_Type_of_Control->setText("Not used for this control mode");
        ui->labelname_Angle1->setText("N/A");
        ui->labelname_Angle2->setText("N/A");
        ui->labelname_B_magnitude->setText("N/A");
        ui->labelname_factor->setText("N/A");
    }
}
//------------------------------------------------------------------------
//
void MainWindow::setTiltSlot(void)
{
    // GUI in degrees. Computation in radians
    tilt = (ui->doubleSpinBox_tilt->value()) * M_PI / 180.0;
}
//------------------------------------------------------------------------
//
void MainWindow::setRollSlot(void)
{
    // GUI in degrees. Computation in radians
    roll = (ui->doubleSpinBox_roll->value()) * M_PI / 180.0;

    // Change desired global field as relation angle is changing
    tip2global(tilt, roll, B_Desired, B_Global_Desired);
    // Find best motor angle solution to desired field
    find_angles_3(pAct, RzyAct, mAct, pTool, B_Global_Desired, K, init_Angles, next_Angles);
    // Determine the resulting actual field
    find_Field(pAct, mAct, RzyAct, init_Angles, B_Global_Output);
    // Write angles to motors
    ArduinoWrite4(next_Angles);
    updateAngles();

    // Send roll value to the arduino to initiate roll
    std::string writeport = "COM7";

    int nVal = (int) ( (ui->doubleSpinBox_roll->value())*(EncoderStepConversionTool/2)/180 + 50000 );
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string position = std::to_string( nVal );
    position = 'r' + position + ',';
    arduino << position;
    // cout << position;
    arduino.close();

}

void MainWindow::setFactorSlot(void)
{
    k_factor = (ui->doubleSpinBox_factor->value()) / 1000;
}

void MainWindow::ResetMotorSlot(void)
{
    for (int i=0; i<4; i++)
    {
        init_Angles[i] = M_PI/2;
        if (i < 3)
            B_Global_Output[i] = 0.0;
    }
    ArduinoWrite4(init_Angles);
}

void MainWindow::clearTimeSlot(void)
{
    currentTime = 0.0;
    qv_t_mag.clear();
    qv_Bx_mag.clear();
    qv_By_mag.clear();
    qv_Bz_mag.clear();
    qv_t_ang.clear();
    qv_motor0.clear();
    qv_motor1.clear();
    qv_motor2.clear();
    qv_motor3.clear();
}

void MainWindow::changeRoll(int dir)
{
    double temp_val = (ui->doubleSpinBox_roll->value());
//    if ( abs(temp_val) > ui->doubleSpinBox_roll->maximum() ) not necessary
//    {
//        dir = 0;
//        ui->doubleSpinBox_roll->setValue(temp_val/abs(temp_val) * ui->doubleSpinBox_roll->maximum() );
//    }
//    else
//    {
//        ui->doubleSpinBox_roll->setValue(temp_val+0.5*dir);
//    }
    ui->doubleSpinBox_roll->setValue(temp_val+0.5*dir);

    if (dir!=0)
    {
        setRollSlot();
    }
}
