
#ifndef SERIAL_H
#define SERIAL_H

class Serial
{
public:
    virtual char getc(void){return 0;};
    virtual void putc(char c){};
    virtual void write(const char *buffer, int len){};
    virtual void write(const char *string){};
    virtual bool available(void){return false;};
    virtual void listen(bool enable){};
    virtual int readUntil(char *buf, int len, char stopChar, int timeout_ms){return 0;};
    virtual void setBaudrate(int baud){};
};

#endif // SERIAL_H
