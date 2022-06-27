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

public:
    float setpoint;
    PID(float kp, float ki, float kd);
    PID(float kp, float ki, float kd, float minout, float maxout);
    ~PID();
    /**********************************************************************/
    void setOutputRange(float min, float max);
    float compute(float input);
};

#endif // !_PID_H_