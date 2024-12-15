#include "KalmanFilter.h"
#include <math.h>

void SimpleKalmanFilter_Init(SimpleKalmanFilter *filter, float mea_e, float est_e, float q)
{
    filter->err_measure = mea_e;
    filter->err_estimate = est_e;
    filter->q = q;
    filter->current_estimate = 0;
    filter->last_estimate = 0;
    filter->kalman_gain = 0;
}

float SimpleKalmanFilter_updateEstimate(SimpleKalmanFilter *filter, float mea)
{
    filter->kalman_gain = filter->err_estimate / (filter->err_estimate + filter->err_measure);
    filter->current_estimate = filter->last_estimate + filter->kalman_gain * (mea - filter->last_estimate);
    filter->err_estimate = (1.0f - filter->kalman_gain) * filter->err_estimate + fabsf(filter->last_estimate - filter->current_estimate) * filter->q;
    filter->last_estimate = filter->current_estimate;

    return filter->current_estimate;
}

void SimpleKalmanFilter_setMeasurementError(SimpleKalmanFilter *filter, float mea_e)
{
    filter->err_measure = mea_e;
}

void SimpleKalmanFilter_setEstimateError(SimpleKalmanFilter *filter, float est_e)
{
    filter->err_estimate = est_e;
}

void SimpleKalmanFilter_setProcessNoise(SimpleKalmanFilter *filter, float q)
{
    filter->q = q;
}

float SimpleKalmanFilter_getKalmanGain(SimpleKalmanFilter *filter)
{
    return filter->kalman_gain;
}

float SimpleKalmanFilter_getEstimateError(SimpleKalmanFilter *filter)
{
    return filter->err_estimate;
}