#include "PID.h"

PID::PID(float kp, float ki, float kd)
{
    _kd = kd;
    _ki = ki;
    _kp = kp;
    _minOut = 0;
    _maxOut = 100;
    outputSum = 0;
    lastInput = 0;
}

PID::PID(float kp, float ki, float kd, float minout, float maxout)
{
    _kd = kd;
    _ki = ki;
    _kp = kp;
    _minOut = minout;
    _maxOut = maxout;
    outputSum = 0;
    lastInput = 0;
}

void PID::setOutputRange(float min, float max)
{
    _minOut = min;
    _maxOut = max;
}

float PID::compute(float input)
{
    float output = 0;
    float err = setpoint - input;
    float dInput = (input - lastInput);

    outputSum += (_ki * err);
    if (outputSum > _maxOut)
        outputSum = _maxOut;

    if (outputSum < _minOut)
        outputSum = _minOut;

    output = _kp * err + outputSum - _kd * dInput;
    if (output > _maxOut)
        output = _maxOut;

    if (output < _minOut)
        output = _minOut;

    return output;
}