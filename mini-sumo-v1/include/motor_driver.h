
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

class MotorDriver
{
private:
    float speedA;
    float speedB;

    bool slowDecayA;
    bool slowDecayB;

    uint pinPWMA;
    uint pinPWMB;
    uint pinDIRA;
    uint pinDIRB;
    uint pinINVA;
    uint pinINVB;

    uint slice_num_A;
    uint chan_A;
    uint slice_num_B;
    uint chan_B;

    uint frequency;
    uint16_t pwmPeriod;
public:
    MotorDriver(uint pinPWMA, uint pinPWMB, uint pinDIRA, uint pinDIRB, uint pinINVA, uint pinINVB, uint frequency = 20000, bool slowDecayA = true, bool slowDecayB = true);
    void init(void);
    void enable(void);
    void disable(void);
    void stop(void);
    void setSpeedA(float speed);
    void setSpeedB(float speed);
    void setSpeed(float A, float B);
    void setDecayA(bool slow);
    void setDecayB(bool slow);
    void setFrequency(uint frequency);
};

#endif // MOTOR_DRIVER_H
