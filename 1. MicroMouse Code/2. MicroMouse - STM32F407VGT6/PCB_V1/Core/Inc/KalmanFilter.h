#ifndef SIMPLEKALMANFILTER_H
#define SIMPLEKALMANFILTER_H

#include <stdio.h>
#include <math.h>

typedef struct
{
    float err_measure;
    float err_estimate;
    float q;
    float current_estimate;
    float last_estimate;
    float kalman_gain;
} SimpleKalmanFilter;

void SimpleKalmanFilter_Init(SimpleKalmanFilter *filter, float mea_e, float est_e, float q);
float SimpleKalmanFilter_updateEstimate(SimpleKalmanFilter *filter, float mea);
void SimpleKalmanFilter_setMeasurementError(SimpleKalmanFilter *filter, float mea_e);
void SimpleKalmanFilter_setEstimateError(SimpleKalmanFilter *filter, float est_e);
void SimpleKalmanFilter_setProcessNoise(SimpleKalmanFilter *filter, float q);
float SimpleKalmanFilter_getKalmanGain(SimpleKalmanFilter *filter);
float SimpleKalmanFilter_getEstimateError(SimpleKalmanFilter *filter);

#endif