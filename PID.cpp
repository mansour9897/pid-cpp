#include "PID.h"
#include "pico/stdlib.h"
#include <stdio.h>
PID::PID(float kp, float ki, float kd)
    : PID::PID(kp, ki, kd, 0, 100)
{
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
    sampleTime = 100;
}

void PID::setOutputRange(float min, float max)
{
    _minOut = min;
    _maxOut = max;
}

float PID::compute(float input)
{
    if (to_ms_since_boot(get_absolute_time()) - lastTime < sampleTime)
        return output;

    lastTime = to_ms_since_boot(get_absolute_time());
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

    printf("set: %f\tin: %f\terr: %f\tout: %f\r", setpoint, input, err, output);
    return output;
}

void PID::clear()
{
    output=0;
    lastInput=0;
    outputSum=0;
}
void PID::setSampleTime(unsigned long t)
{
    sampleTime = t;
}

PID::~PID() {}