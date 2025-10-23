
#include <stdio.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "one_wire_device.h"

OneWireDevice::OneWireDevice(OneWire &oneWire, uint index) : oneWire(oneWire), index(index)
{
}

bool OneWireDevice::begin(void)
{
    return waitForBoot(1000);
}

bool OneWireDevice::waitForBoot(int timeout_ms)
{
    absolute_time_t start = get_absolute_time();
    do
    {
        if (readRegister(0x00, nullptr))
        {
            return true;
        }
        sleep_ms(1);
    } while (absolute_time_diff_us(start, get_absolute_time()) < timeout_ms * 1000);
    return false;
}

bool OneWireDevice::setIndex(uint index)
{
    uint _index = this->index;
    if (!writeRegister(0x15, index))
    {
        return false;
    }
    this->index = index;
    saveConfiguration();
    if (!waitForBoot(1000))
    {
        this->index = _index;
        return false;
    }
    return true;
}

bool OneWireDevice::readIndex(uint &index)
{
    if (!readRegister(0x15, &index))
    {
        return false;
    }
    return true;
}

bool OneWireDevice::setIOMode(OneWireDevice::DeviceIOMode mode)
{
    if (mode > IOMODE_PWM || mode < IOMODE_SERIAL)
    {
        return false;
    }
    return writeRegister(0x14, mode);
}

OneWireDevice::DeviceIOMode OneWireDevice::getIOMode(void)
{
    // Not implemented
    uint mode;
    if (!readRegister(0x14, &mode))
    {
        return IOMODE_ERROR;
    }
    if (mode > IOMODE_PWM || mode < IOMODE_SERIAL)
    {
        return IOMODE_ERROR;
    }
    return (DeviceIOMode)mode;
}

bool OneWireDevice::setSerialBaudRate(OneWireDevice::DeviceSerialBaudRate baud)
{
    if (baud > SERIAL_BAUD_250000 || baud < SERIAL_BAUD_9600)
    {
        return false;
    }

    DeviceSerialBaudRate previousBaud = getSerialBaudRate();

    if (!writeRegister(0x16, baud))
    {
        return false;
    }
    saveConfiguration();
    
    oneWire.setBaudrate(baudRateFromCode(baud));

    if (!waitForBoot(1000))
    {
        oneWire.setBaudrate(baudRateFromCode(previousBaud));
        return false;
    }
    return true;
}

OneWireDevice::DeviceSerialBaudRate OneWireDevice::getSerialBaudRate(void)
{
    uint baud;
    if (!readRegister(0x16, &baud))
    {
        return SERIAL_BAUD_ERROR;
    }
    if (baud > SERIAL_BAUD_250000 || baud < SERIAL_BAUD_9600)
    {
        return SERIAL_BAUD_ERROR;
    }
    return (DeviceSerialBaudRate)baud;
}

/*! \brief Read a value from a device register
 * \param reg Register address
 * \param value Pointer to variable to receive the value, or nullptr if the value is not required
 * \return true if the command was acknowledged and a value read, false on error
 */
bool OneWireDevice::readRegister(uint reg, uint *value)
{
    char buffer[7]; // "R" + 2*index + 2*reg + "\n" + "\0"
    snprintf(buffer, sizeof(buffer), "R%02X%02X\n", index, reg);
    oneWire.write(buffer);
    int n = oneWire.read(buffer, sizeof(buffer));
    if (n <= 0 || n > 4)
    {
        return false;
    }
    if (value == nullptr)
    {
        return true;
    }
    *value = (uint)strtol(buffer, NULL, 16);
    return true;
}

/*! \brief Write a value to a device register
 * \param reg Register address
 * \param value Value to write
 * \return true if the command was acknowledged, false on error
 */
bool OneWireDevice::writeRegister(uint reg, uint value)
{
    char buffer[11]; // "W" + 2*index + 2*reg + 4*value + "\n" + "\0"
    snprintf(buffer, sizeof(buffer), "W%02X%02X%04X\n", index, reg, value);
    oneWire.write(buffer);
    int n = oneWire.read(buffer, sizeof(buffer));
    return n == 1 && buffer[0] == 'A';
}

/*! \brief Save the current device settings to non-volatile memory
 * \return true if the command was acknowledged, false on error
 */
bool OneWireDevice::saveConfiguration(void)
{
    char buffer[5]; // "S" + 2*index + "\n" + "\0"
    snprintf(buffer, sizeof(buffer), "S%02X\n", index);
    oneWire.write(buffer);
    int n = oneWire.read(buffer, sizeof(buffer));
    return n == 1 && buffer[0] == 'A';
}

/*! \brief Reset the device to factory default settings
 * \return true if the command was acknowledged, false on error
 */
bool OneWireDevice::resetConfiguration(void)
{
    char buffer[5]; // "Z" + 2*index + "\n" + "\0"
    snprintf(buffer, sizeof(buffer), "Z%02X\n", index);
    oneWire.write(buffer);
    int n = oneWire.read(buffer, sizeof(buffer));
    return n == 1 && buffer[0] == 'A';
}