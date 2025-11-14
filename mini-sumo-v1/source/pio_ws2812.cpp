
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "pio_ws2812.h"

WS2812_Pio::WS2812_Pio(PIO pio, uint sm, uint pin, float frequency, bool rgbw) : pio(pio), sm(sm), pin(pin), frequency(frequency), rgbw(rgbw)
{
    color = 0;
    blinking = false;
}

void WS2812_Pio::init(void)
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    uint offset = pio_add_program(pio, &ws2812_program);
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = clock_get_hz(clk_sys) / (frequency * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    pio_sm_put_blocking(pio, sm, 0);
}

void WS2812_Pio::set(uint32_t grbw)
{
    this->color_off = Color(grbw);
    blinking = false;
}

void WS2812_Pio::set(Color c)
{
    this->color_off = c;
    blinking = false;
}

void WS2812_Pio::setBlocking(Color c)
{
    this->color_off = c;
    blinking = false;
    update();
}

void WS2812_Pio::_set(uint32_t grbw)
{
    if (color == grbw) return;
    color = grbw;
    pio_sm_put_blocking(pio, sm, grbw << 8u);
}

void WS2812_Pio::update(void)
{
    if (blinking)
    {
        if (state)
        {
            if (absolute_time_diff_us(timestamp, get_absolute_time()) > period_on)
            {
                timestamp = get_absolute_time();
                state = false;
                _set(color_off.toGRBW_u32());

                if (count == 0)
                {
                    blinking = false;
                }
            }
        }
        else
        {
            if (absolute_time_diff_us(timestamp, get_absolute_time()) > period_off)
            {
                timestamp = get_absolute_time();
                state = true;
                _set(color_on.toGRBW_u32());

                if (count > 0)
                {
                    count--;
                }
            }
        }
    }
    else
    {
        _set(color_off.toGRBW_u32());
    }
}

void WS2812_Pio::blink(Color color_on, Color color_off, uint period_on, uint period_off, int count)
{
    this->color_on = color_on;
    this->color_off = color_off;
    this->period_on = period_on * 1000;
    this->period_off = period_off * 1000;
    this->count = count;
    state = false;
    timestamp = 0;
    blinking = true;
}

void WS2812_Pio::blinkBlocking(Color color_on, Color color_off, uint period_on, uint period_off, int count)
{
    if (count < 0) return;
    blink(color_on, color_off, period_on, period_off, count);

    while (this->count > 0)
    {
        update();
    }
}
