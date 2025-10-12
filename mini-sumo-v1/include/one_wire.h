
#ifndef ONE_WIRE_H
#define ONE_WIRE_H

#include "serial.h"

class OneWire
{
private:
    Serial &serial;
    char terminator;
    int timeout_ms;

public:
    OneWire(Serial &serial, int timeout_ms = 50, char terminator = '\n');
    void init(void);
    void setBaudrate(int baud);
    int getBaudrate(void);
    void write(char *string);
    void write(char *buffer, int length);
    int read(char *buffer, int maxLength);
};

#endif // ONE_WIRE_H
