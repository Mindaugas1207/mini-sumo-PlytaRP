
#include <stdio.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "one_wire.h"

OneWire::OneWire(Serial &serial, int timeout_ms, char terminator) : serial(serial), terminator(terminator), timeout_ms(timeout_ms)
{
}
void OneWire::init(void)
{
}
void OneWire::setBaudrate(int baud)
{
    serial.setBaudrate(baud);
}
int OneWire::getBaudrate(void)
{
    // Not implemented
    return 0;
}
void OneWire::write(const char *string)
{
    serial.listen(false);
    serial.write(string);
}
void OneWire::write(const char *buffer, int length)
{
    serial.listen(false);
    serial.write(buffer, length);
}
int OneWire::read(char *buffer, int maxLength)
{
    serial.listen(true);
    int n = serial.readUntil(buffer, maxLength, terminator, timeout_ms);
    serial.listen(false);
    n--;
    if (n < 0 || buffer[n] != terminator)
    {
        return 0;
    }
    buffer[n] = '\0';
    return n;
}