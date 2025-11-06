
#include <cstdlib>
#include "pico/stdlib.h"
#include "ir_receiver.h"

#define RCV_NFIFO_REG (0x00)
#define RCV_FIFO_DATA_REG (0x01)

IrReceiver::IrReceiver(OneWire &oneWire, uint index) : OneWireDevice(oneWire, index)
{
}

int IrReceiver::getFIFOCount(void)
{
    uint count;
    if (!readRegister(RCV_NFIFO_REG, &count))
    {
        return -1;
    }
    return (int)count;
}

int IrReceiver::getFIFORead(void)
{
    uint cmd;
    if (!readRegister(RCV_FIFO_DATA_REG, &cmd))
    {
        return -1;
    }
    return (int)cmd;
}

bool IrReceiver::getCommand(uint* command)
{
    int cmd;
    if (getFIFOCount() <= 0) return false;
    cmd = getFIFORead();
    if (cmd < 0) return false;
    *command = (uint)cmd;
    return true;
}

uint IrReceiver::getCommandBlocking(void)
{
    bool ok = false;
    uint command = 0;
    do
    {
        ok = getCommand(&command);
    }
    while (!ok);

    return command;
}
