


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
    pio_sm_config c_tx;
    pio_sm_config c_rx;
    uint offset_tx;
    uint offset_rx;
public:
    PioOneWireSerial(PIO pio, uint sm_tx, uint sm_rx, uint pin, uint baud);
    void init(void);
    void deinit(void);
    void stop(void);
    void start(void);
    void changePin(uint pin, bool forced = false) override;
    void listen(bool enable) override; 
    bool available(void) override;
    char getc(void) override;
    void putc(char c) override;
    void write(const char *buffer, int len) override;
    void write(const char *string) override;
    int readUntil(char *buf, int len, char stopChar, int timeout_ms) override;
    void setBaudrate(int baud) override;
    uint getBaudrate(void) override;
};

#endif // PIO_ONE_WIRE_SERIAL_H
