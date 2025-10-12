
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
    pio_sm_set_pins_with_mask64(pio, sm_tx, 0ull << pin, 1ull << pin);
    pio_sm_set_pindirs_with_mask64(pio, sm_tx, 0ull << pin, 1ull << pin);
    pio_gpio_init(pio, pin);

    uint offset_tx = pio_add_program(pio, &PioOneWireSerial_tx_program);
    uint offset_rx = pio_add_program(pio, &PioOneWireSerial_rx_program);
    pio_sm_config c_tx = PioOneWireSerial_tx_program_get_default_config(offset_tx);
    pio_sm_config c_rx = PioOneWireSerial_rx_program_get_default_config(offset_rx);
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);

    sm_config_set_clkdiv(&c_tx, div);
    sm_config_set_clkdiv(&c_rx, div);

    sm_config_set_out_shift(&c_tx, true, false, 32);
    sm_config_set_out_pins(&c_tx, pin, 1);
    sm_config_set_sideset_pins(&c_tx, pin);
    sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX);

    sm_config_set_in_pins(&c_rx, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c_rx, pin); // for JMP
    sm_config_set_in_shift(&c_rx, true, false, 32);
    sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm_tx, offset_tx, &c_tx);
    pio_sm_init(pio, sm_rx, offset_rx, &c_rx);

    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_enabled(pio, sm_rx, false);
}
void PioOneWireSerial::listen(bool enable)
{
    if (enable)
    {
        pio_sm_restart(pio, sm_rx);
    }
    pio_sm_set_enabled(pio, sm_rx, enable);
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
void PioOneWireSerial::write(char *buffer, int len)
{
    for (int i = 0; i < len; i++)
    {
        putc(buffer[i]);
    }
}
void PioOneWireSerial::write(char *string)
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
            if (absolute_time_diff_us(start, get_absolute_time()) > timeout_ms * 1000)
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
