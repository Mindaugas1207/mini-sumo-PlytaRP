
#ifndef PIO_WS2812_H
#define PIO_WS2812_H

#include "led.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

class WS2812_Pio : public Led
{
private:
    PIO pio;
    uint sm;
    uint pin;
    float frequency;
    bool rgbw;

    absolute_time_t timestamp;
    Color color_on;
    Color color_off;
    uint32_t color;
    uint period_on;
    uint period_off;
    int count;

    bool blinking;
    bool state;
    void _set(uint32_t grbw);
public:
    WS2812_Pio(PIO pio, uint sm, uint pin, float frequency = 800000, bool rgbw = false);
    void init(void);
    void set(uint32_t grbw);
    void set(Color c) override;

    void update(void);
    void blink(Color color_on, Color color_off, uint period_on, uint period_off, int count = -1) override;
    void blinkBlocking(Color color_on, Color color_off, uint period_on, uint period_off, int count) override;
};

#endif // PIO_WS2812_H
