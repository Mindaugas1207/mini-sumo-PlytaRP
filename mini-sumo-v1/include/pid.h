
#ifndef INC_PID_H_
#define INC_PID_H_

#include "pico/stdlib.h"
#include <math.h>
#include <algorithm>
#include <limits>
#include <vmath.h>

class PID
{
private:
    double lastInput;
    double integral;

public:
    double kp;
    double ki;
    double kd;
    double integralLimit;
    double minError;

    PID(double kp, double ki, double kd, double integralLimit = std::numeric_limits<double>::infinity(), double minError = 0) : kp(kp), ki(ki), kd(kd), integralLimit(integralLimit), minError(minError)
    {
    }

    double compute(double setpoint, double input, double dt)
    {
        double error = setpoint - input;
        if (std::abs(error) < minError) error = 0;
        integral = std::clamp(integral + error * ki * dt, -integralLimit, integralLimit);
        double derivative = (input - lastInput) * kd / dt;
        double output = std::clamp(kp * error + integral + derivative, -1.0, 1.0);
        lastInput = input;
        return output;
    }

    double compute_radians(double setpoint, double input, double dt)
    {
        double error = vmath::UnwrapAngle(setpoint - input); //[-pi, +pi]
        if (std::abs(error) < minError) error = 0;
        integral = std::clamp(integral + error * ki * dt, -integralLimit, integralLimit); //[-1, +1]
        double derivative = vmath::UnwrapAngle(input - lastInput) * kd / dt; //[-1, +1]
        double output = std::clamp(kp * error + integral + derivative, -1.0, 1.0); //[-1, +1]
        lastInput = input;
        return output;
    }

    void reset(void)
    {
        lastInput = 0;
        integral = 0;
    }
};

class Ramp
{
private:
    absolute_time_t timestamp;
    double coef;
    double rate;
    double min;
public:
    Ramp(double coeficient, double rate, double min) : coef(coeficient), rate(rate), min(min)
    {
        timestamp = get_absolute_time();
    }

    double compute(double current, double target)
    {
        absolute_time_t t = get_absolute_time();
        int64_t dif = absolute_time_diff_us(timestamp, t);
        timestamp = t;
        double d = std::abs(target - current);
        //printf("%f\n", d);
        
        if (std::abs(current) < (min - 0.01))
        {
            if (std::abs(target) < (min - 0.01))
            {
                current = 0;
            }
            else
            {
                if (target < 0) current = -min;
                else current = min;
            }
        }
        else if (d < 0.01) 
        {
            current = target;
        }
        else
        {
            if (d < rate) d = rate;
            double inc = d * dif * coef;
            if (target > current)
            {
                current += inc;
                if (current > target) current = target;
            }
            else if (target < current)
            {
                current -= inc;
                if (current < target) current = target;
            }
        }

        return current;
    }

    void reset()
    {
        timestamp = get_absolute_time();
    }
};

#endif // INC_PID_H_