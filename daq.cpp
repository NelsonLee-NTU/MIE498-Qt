#include "daq.h"
#include "NIDAQmx.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>

//QString filename = "C:/Users/MicroRoboticsLab/Documents/GitHub/PermanentMagnetSystemApplication_New/Data.txt";
QString filename = "c:/Users/MicroRoboticsLab/Documents/data0317(2).txt";

void data_acquisition(double angles[4], float x_position, float y_position, float z_position)
{ 
    TaskHandle taskHandle_x = 0;
    int32 samplesReceived_x = 0;
    float64 data_x[1000];

    DAQmxCreateTask("MyTask_x", &taskHandle_x);
    DAQmxCreateAIVoltageChan(taskHandle_x, "Dev1/ai1", "", DAQmx_Val_Cfg_Default, -5, 5, DAQmx_Val_Volts, NULL);
    DAQmxCfgSampClkTiming(taskHandle_x, "", 10000.0, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, 1000);
    DAQmxStartTask(taskHandle_x);
    DAQmxReadAnalogF64(taskHandle_x, 1000, 10.0, DAQmx_Val_GroupByChannel, data_x, 1000, &samplesReceived_x, NULL);
    DAQmxStopTask(taskHandle_x);
    DAQmxClearTask(taskHandle_x);

    TaskHandle taskHandle_y = 0;
    int32 samplesReceived_y = 0;
    float64 data_y[1000];

    DAQmxCreateTask("MyTask_y", &taskHandle_y);
    DAQmxCreateAIVoltageChan(taskHandle_y, "Dev1/ai2", "", DAQmx_Val_Cfg_Default, -5, 5, DAQmx_Val_Volts, NULL);
    DAQmxCfgSampClkTiming(taskHandle_y, "", 10000.0, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, 1000);
    DAQmxStartTask(taskHandle_y);
    DAQmxReadAnalogF64(taskHandle_y, 1000, 10.0, DAQmx_Val_GroupByChannel, data_y, 1000, &samplesReceived_y, NULL);
    DAQmxStopTask(taskHandle_y);
    DAQmxClearTask(taskHandle_y);

    TaskHandle taskHandle_z = 0;
    int32 samplesReceived_z = 0;
    float64 data_z[1000];

    DAQmxCreateTask("MyTask_z", &taskHandle_z);
    DAQmxCreateAIVoltageChan(taskHandle_z, "Dev1/ai3", "", DAQmx_Val_Cfg_Default, -5, 5, DAQmx_Val_Volts, NULL);
    DAQmxCfgSampClkTiming(taskHandle_z, "", 10000.0, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, 1000);
    DAQmxStartTask(taskHandle_z);
    DAQmxReadAnalogF64(taskHandle_z, 1000, 10.0, DAQmx_Val_GroupByChannel, data_z, 1000, &samplesReceived_z, NULL);
    DAQmxStopTask(taskHandle_z);
    DAQmxClearTask(taskHandle_z);

    float64 sum_x = 0;
    for (int i=0; i<1000; i++)
        sum_x += data_x[i];

    float64 sum_y = 0;
    for (int i=0; i<1000; i++)
        sum_y += data_y[i];

    float64 sum_z = 0;
    for (int i=0; i<1000; i++)
        sum_z += data_z[i];

//    qDebug() << sum_x/1000;
    // Print results
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream stream(&file);
        stream << "location" << " " << x_position << " " << y_position << " " << z_position << " "
               << "angles "<<angles[3]<<" "<< angles[2] <<" "<<angles[1]<<" "<<angles[0]<<" data "
               << sum_x/1000 << " " << sum_y/1000 << " " << sum_z/1000 << endl;
        file.close();
    }
}
