


#ifndef PIO_ONE_WIRE_SERIAL_H
#define PIO_ONE_WIRE_SERIAL_H

#include "serial.h"
#include "hardware/pio.h"

class PioOneWireSerial : public Serial
{
private:
    PIO pio;
    uint sm_tx;
    uint sm_rx;
    uint pin;
    uint baud;

public:
    PioOneWireSerial(PIO pio, uint sm_tx, uint sm_rx, uint pin, uint baud);
    void init(void);
    void listen(bool enable) override; 
    bool available(void) override;
    char getc(void) override;
    void putc(char c) override;
    void write(char *buffer, int len) override;
    void write(char *string) override;
    int readUntil(char *buf, int len, char stopChar, int timeout_ms) override;
};

#endif // PIO_ONE_WIRE_SERIAL_H
