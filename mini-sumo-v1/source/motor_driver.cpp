#include "pico/stdlib.h"
#include "system.h"
#include "motor_driver.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <hardware/pwm.h>
#include <algorithm>

MotorDriver::MotorDriver(uint pinPWMA, uint pinDIRA, uint pinINVA, uint pinPWMB, uint pinDIRB, uint pinINVB, uint frequency, bool slowDecayA, bool slowDecayB, bool invertA, bool invertB) : pinPWMA(pinPWMA), pinDIRA(pinDIRA), pinINVA(pinINVA), pinPWMB(pinPWMB), pinDIRB(pinDIRB), pinINVB(pinINVB), frequency(frequency), slowDecayA(slowDecayA), slowDecayB(slowDecayB), invertA(invertA), invertB(invertB)
{
    slice_num_A = pwm_gpio_to_slice_num(pinPWMA);
    slice_num_B = pwm_gpio_to_slice_num(pinPWMB);
    chan_A = pwm_gpio_to_channel(pinPWMA);
    chan_B = pwm_gpio_to_channel(pinPWMB);
}
void MotorDriver::init(void)
{
    disable();
    //setDecayA(false);
    //setDecayB(false);

    gpio_set_function(pinPWMA, GPIO_FUNC_PWM);
    gpio_set_function(pinPWMB, GPIO_FUNC_PWM);

    gpio_set_function(pinDIRA, GPIO_FUNC_SIO);
    //gpio_set_function(pinINVA, GPIO_FUNC_SIO);
    gpio_set_function(pinDIRB, GPIO_FUNC_SIO);
    //gpio_set_function(pinINVB, GPIO_FUNC_SIO);

    gpio_set_dir(pinDIRA, GPIO_OUT);
    gpio_set_dir(pinDIRB, GPIO_OUT);
    //gpio_set_dir(pinINVA, GPIO_OUT);
    //gpio_set_dir(pinINVB, GPIO_OUT);

    setFrequency(frequency);
    enable();
}
void MotorDriver::enable(void)
{
    pwm_set_enabled(slice_num_A, true);
    pwm_set_enabled(slice_num_B, true);
}
void MotorDriver::disable(void)
{
    stop();
    pwm_set_enabled(slice_num_A, false);
    pwm_set_enabled(slice_num_B, false);
}
void MotorDriver::stop(void)
{
    setSpeed(0, 0);
}
void MotorDriver::setSpeedA(float speed)
{
    speedA = std::clamp(speed, -1.0f, 1.0f);
    uint16_t pwm = std::abs(speedA) * pwmPeriod;

    gpio_put(pinDIRA, speedA >= 0 ? invertA : !invertA);
    pwm_set_chan_level(slice_num_A, chan_A, pwm);
}
void MotorDriver::setSpeedB(float speed)
{
    speedB = std::clamp(speed, -1.0f, 1.0f);
    uint16_t pwm = std::abs(speedB) * pwmPeriod;

    gpio_put(pinDIRB, speedB >= 0 ? invertB : !invertB);
    pwm_set_chan_level(slice_num_B, chan_B, pwm);
}
void MotorDriver::setSpeed(float A, float B)
{
    setSpeedA(A);
    setSpeedB(B);
}
void MotorDriver::setDecayA(bool slow)
{
    slowDecayA = slow;
    gpio_put(pinINVA, slowDecayA ? 1 : 0);
}
void MotorDriver::setDecayB(bool slow)
{
    slowDecayB = slow;
    gpio_put(pinINVB, slowDecayB ? 1 : 0);
}
void MotorDriver::setFrequency(uint frequency)
{
    this->frequency = frequency;
    uint32_t fclock = clock_get_hz(clk_sys);
    float div = (float)fclock / frequency;
    if (div <= 0xFFFF)
    {
        pwmPeriod = (uint16_t)div;
        div = 1.0f;
    }
    else
    {
        pwmPeriod = 0xFFFF;
        div /= 0xFFFF;
    }

    pwm_set_clkdiv(slice_num_A, div);
    pwm_set_clkdiv(slice_num_B, div);
    pwm_set_wrap(slice_num_A, pwmPeriod);
    pwm_set_wrap(slice_num_B, pwmPeriod);
}
