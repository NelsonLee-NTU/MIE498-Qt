#ifndef DAQ_H
#define DAQ_H
#include <stdio.h>
#include <NIDAQmx.h>

void data_acquisition(double angles[4], float x_position, float y_position, float z_position);

#endif // DAQ_H
