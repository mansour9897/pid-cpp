#ifndef _PID_H_
#define _PID_H_

class PID
{
private:
    float _kp;
    float _ki;
    float _kd;
    float _minOut;
    float _maxOut;
    float lastInput;
    float outputSum;
    float output;
    unsigned long sampleTime; // ms
    unsigned long lastTime;   // ms

public:
    float setpoint;
    PID(float kp, float ki, float kd);
    PID(float kp, float ki, float kd, float minout, float maxout);
    ~PID();
    /**********************************************************************/
    void setOutputRange(float min, float max);
    float compute(float input);
    void setSampleTime(unsigned long t);
};

#endif // !_PID_H_