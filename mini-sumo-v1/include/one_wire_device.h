
#ifndef ONE_WIRE_DEVICE_H
#define ONE_WIRE_DEVICE_H

#include "one_wire.h"

class OneWireDevice
{
public:
    enum DeviceIOMode
    {
        IOMODE_ERROR = -1,
        IOMODE_SERIAL = 0,
        IOMODE_DO = 1,
        IOMODE_PWM = 2
    };

    enum DeviceSerialBaudRate
    {
        SERIAL_BAUD_ERROR = -1,
        SERIAL_BAUD_9600 = 0,
        SERIAL_BAUD_19200 = 1,
        SERIAL_BAUD_38400 = 2,
        SERIAL_BAUD_57600 = 3,
        SERIAL_BAUD_74880 = 4,
        SERIAL_BAUD_115200 = 5,
        SERIAL_BAUD_230400 = 6,
        SERIAL_BAUD_250000 = 7
    };
private:
    OneWire &oneWire;
    uint index;

    constexpr int baudRateFromCode(DeviceSerialBaudRate code)
    {
        return code == SERIAL_BAUD_9600     ? 9600
               : code == SERIAL_BAUD_19200  ? 19200
               : code == SERIAL_BAUD_38400  ? 38400
               : code == SERIAL_BAUD_57600  ? 57600
               : code == SERIAL_BAUD_74880  ? 74880
               : code == SERIAL_BAUD_115200 ? 115200
               : code == SERIAL_BAUD_230400 ? 230400
                                            : 250000;
    }

public:
    OneWireDevice(OneWire &oneWire, uint index);

    bool begin(void);

    bool waitForBoot(int timeout_ms);

    bool setIndex(uint index);

    bool readIndex(uint &index);

    uint getIndex(void) const
    {
        return index;
    }

    bool setIOMode(DeviceIOMode mode);

    DeviceIOMode getIOMode(void);

    bool setSerialBaudRate(DeviceSerialBaudRate baud);

    DeviceSerialBaudRate getSerialBaudRate(void);

    /*! \brief Read a value from a device register
     * \param reg Register address
     * \param value Pointer to variable to receive the value, or nullptr if the value is not required
     * \return true if the command was acknowledged and a value read, false on error
     */
    bool readRegister(uint reg, uint *value);

    /*! \brief Write a value to a device register
     * \param reg Register address
     * \param value Value to write
     * \return true if the command was acknowledged, false on error
     */
    bool writeRegister(uint reg, uint value);

    /*! \brief Save the current device settings to non-volatile memory
     * \return true if the command was acknowledged, false on error
     */
    bool saveConfiguration(void);

    /*! \brief Reset the device to factory default settings
     * \return true if the command was acknowledged, false on error
     */
    bool resetConfiguration(void);
};

#endif // ONE_WIRE_DEVICE_H
