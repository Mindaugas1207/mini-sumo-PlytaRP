
#ifndef IR_RECEIVER_H
#define IR_RECEIVER_H

#include "one_wire_device.h"

class IrReceiver : public OneWireDevice
{
public:

    IrReceiver(OneWire &oneWire, uint index);

    int getFIFOCount(void);
    int getFIFORead(void);
    bool getCommand(uint* command);
    uint getCommandBlocking(void);
};

#endif // DISTANCE_SENSOR_H
