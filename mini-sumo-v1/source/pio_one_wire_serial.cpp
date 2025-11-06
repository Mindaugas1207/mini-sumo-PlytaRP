
#include <stdio.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "one_wire.pio.h"
#include "pio_one_wire_serial.h"

PioOneWireSerial::PioOneWireSerial(PIO pio, uint sm_tx, uint sm_rx, uint pin, uint baud) : pio(pio), sm_tx(sm_tx), sm_rx(sm_rx), pin(pin), baud(baud)
{
}
void PioOneWireSerial::init(void)
{
    //pio_sm_set_pins_with_mask64(pio, sm_tx, 0ull << pin, 1ull << pin);
    //pio_sm_set_pindirs_with_mask64(pio, sm_tx, 0ull << pin, 1ull << pin);
    //pio_gpio_init(pio, pin);

    offset_tx = pio_add_program(pio, &PioOneWireSerial_tx_program);
    offset_rx = pio_add_program(pio, &PioOneWireSerial_rx_program);
    c_tx = PioOneWireSerial_tx_program_get_default_config(offset_tx);
    c_rx = PioOneWireSerial_rx_program_get_default_config(offset_rx);
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);

    sm_config_set_clkdiv(&c_tx, div);
    sm_config_set_clkdiv(&c_rx, div);

    sm_config_set_out_shift(&c_tx, true, false, 32);
    sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX);

    sm_config_set_in_shift(&c_rx, true, false, 32);
    sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX);

    pio_gpio_init(pio, pin);
    sm_config_set_out_pins(&c_tx, pin, 1);
    sm_config_set_sideset_pins(&c_tx, pin);
    sm_config_set_in_pins(&c_rx, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c_rx, pin); // for JMP

    pio_sm_init(pio, sm_tx, offset_tx, &c_tx);
    pio_sm_init(pio, sm_rx, offset_rx, &c_rx);

    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_enabled(pio, sm_rx, false);
}

void PioOneWireSerial::deinit(void)
{
    gpio_deinit(pin);
}

void PioOneWireSerial::stop(void)
{
    pio_sm_set_enabled(pio, sm_tx, false);
    pio_sm_set_enabled(pio, sm_rx, false);
}

void PioOneWireSerial::start(void)
{
    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_enabled(pio, sm_rx, false);
}

void PioOneWireSerial::changePin(uint pin)
{
    if (this->pin == pin) return;
    stop();
    gpio_deinit(pin);
    this->pin = pin;
    pio_gpio_init(pio, pin);
    sm_config_set_out_pins(&c_tx, pin, 1);
    sm_config_set_sideset_pins(&c_tx, pin);
    sm_config_set_in_pins(&c_rx, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c_rx, pin); // for JMP
    pio_sm_init(pio, sm_tx, offset_tx, &c_tx);
    pio_sm_init(pio, sm_rx, offset_rx, &c_rx);
    start();
}

void PioOneWireSerial::setBaudrate(int baud)
{
    pio_sm_set_enabled(pio, sm_tx, false);
    pio_sm_set_enabled(pio, sm_rx, false);
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    pio_sm_set_clkdiv(pio, sm_tx, div);
    pio_sm_set_clkdiv(pio, sm_rx, div);
    pio_sm_restart(pio, sm_tx);
    pio_sm_restart(pio, sm_rx);
    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_enabled(pio, sm_rx, false);
    this->baud = baud;
}

void PioOneWireSerial::listen(bool enable)
{
    if (enable)
    {
        uint32_t SM_STALL_MASK = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm_tx);
        pio->fdebug |= SM_STALL_MASK;   
        while (!(pio->fdebug & SM_STALL_MASK)); {} //wait for tx to stall
        pio_sm_set_enabled(pio, sm_rx, true);
    }
    else
    {
        pio_sm_set_enabled(pio, sm_rx, false);
    }
}
bool PioOneWireSerial::available(void)
{
    return !pio_sm_is_rx_fifo_empty(pio, sm_rx);
}
char PioOneWireSerial::getc(void)
{
    io_rw_8 *rxfifo_shift = (io_rw_8 *)&pio->rxf[sm_rx] + 3;
    return (char)*rxfifo_shift;
}
void PioOneWireSerial::putc(char c)
{
    pio_sm_put_blocking(pio, sm_tx, (uint32_t)c);
}
void PioOneWireSerial::write(const char *buffer, int len)
{
    for (int i = 0; i < len; i++)
    {
        putc(buffer[i]);
    }
}
void PioOneWireSerial::write(const char *string)
{
    while (*string)
    {
        putc(*string++);
    }
}
int PioOneWireSerial::readUntil(char *buf, int len, char stopChar, int timeout_ms)
{
    int count = 0;
    absolute_time_t start = get_absolute_time();
    while (count < len)
    {
        while (!available())
        {
            if ((timeout_ms != -1) && (absolute_time_diff_us(start, get_absolute_time()) > (timeout_ms * 1000)))
            {
                return count;
            }
            tight_loop_contents();
        }
        char c = getc();
        buf[count++] = c;
        if (c == stopChar)
        {
            break;
        }
    }
    return count;
}
