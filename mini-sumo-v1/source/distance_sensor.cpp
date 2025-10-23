
#include <cstdlib>
#include "pico/stdlib.h"
#include "distance_sensor.h"

DistanceSensor::DistanceSensor(OneWire &oneWire, uint index) : OneWireDevice(oneWire, index)
{
}
DistanceSensor::RangeFlags DistanceSensor::getRangeFlags(void)
{
    uint flags;
    if (!readRegister(0x00, &flags))
    {
        return (RangeFlags)0;
    }
    return (RangeFlags)flags;
}
int DistanceSensor::getDistance_mm(void)
{
    uint distance;
    if (!readRegister(0x01, &distance))
    {
        return -1;
    }
    return (int)distance;
}
int DistanceSensor::getDistance_cm(void)
{
    uint distance;
    if (!readRegister(0x02, &distance))
    {
        return -1;
    }
    return (int)distance;
}
int DistanceSensor::getSPADCount(void)
{
    uint spadCount;
    if (!readRegister(0x03, &spadCount))
    {
        return -1;
    }
    return (int)spadCount;
}
int DistanceSensor::getSignalRate_MCPS(void)
{
    uint signalRate;
    if (!readRegister(0x04, &signalRate))
    {
        return -1;
    }
    return (int)signalRate;
}
int DistanceSensor::getAmbientRate_MCPS(void)
{
    uint ambientRate;
    if (!readRegister(0x05, &ambientRate))
    {
        return -1;
    }
    return (int)ambientRate;
}
int DistanceSensor::getSignalSigma_mm(void)
{
    uint signalSigma;
    if (!readRegister(0x06, &signalSigma))
    {
        return -1;
    }
    return (int)signalSigma;
}
DistanceSensor::RangeFlags DistanceSensor::getRangeChecks(void)
{
    uint flags;
    if (!readRegister(0x0C, &flags))
    {
        return (RangeFlags)0;
    }
    return (RangeFlags)flags;
}
int DistanceSensor::getRangeMax_mm(void)
{
    uint maxDistance;
    if (!readRegister(0x0D, &maxDistance))
    {
        return -1;
    }
    return (int)maxDistance;
}
int DistanceSensor::getRangeMin_mm(void)
{
    uint minDistance;
    if (!readRegister(0x0E, &minDistance))
    {
        return -1;
    }
    return (int)minDistance;
}
int DistanceSensor::getRangeTiming_ms(void)
{
    uint timing;
    if (!readRegister(0x0F, &timing))
    {
        return -1;
    }
    return (int)timing;
}
int DistanceSensor::getRangeOffset(void)
{
    uint offset;
    if (!readRegister(0x10, &offset))
    {
        return -1;
    }
    return (int)offset;
}
int DistanceSensor::getRangeXTALK(void)
{
    uint xtalk;
    if (!readRegister(0x11, &xtalk))
    {
        return -1;
    }
    return (int)xtalk;
}
int DistanceSensor::getRangeSignalLimit_MCPS(void)
{
    uint limit;
    if (!readRegister(0x12, &limit))
    {
        return -1;
    }
    return (int)limit;
}
int DistanceSensor::getRangeSigmaLimit_mm(void)
{
    uint limit;
    if (!readRegister(0x13, &limit))
    {
        return -1;
    }
    return (int)limit;
}
bool DistanceSensor::setRangeChecks(DistanceSensor::RangeFlags flags)
{
    return writeRegister(0x0C, flags);
}
bool DistanceSensor::setRangeMax_mm(uint maxDistance)
{
    return writeRegister(0x0D, maxDistance);
}
bool DistanceSensor::setRangeMin_mm(uint minDistance)
{
    return writeRegister(0x0E, minDistance);
}
bool DistanceSensor::setRangeTiming_ms(uint timing)
{
    return writeRegister(0x0F, timing);
}
bool DistanceSensor::setRangeOffset(uint offset)
{
    return writeRegister(0x10, offset);
}
bool DistanceSensor::setRangeXTALK(uint xtalk)
{
    return writeRegister(0x11, xtalk);
}
bool DistanceSensor::setRangeSignalLimit_MCPS(uint limit)
{
    return writeRegister(0x12, limit);
}
bool DistanceSensor::setRangeSigmaLimit_mm(uint limit)
{
    return writeRegister(0x13, limit);
}
bool DistanceSensor::setDetectionMode(DistanceSensor::DetectionMode mode)
{
    if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_ANY_VALID_RANGE)
    {
        return false;
    }
    return writeRegister(0x08, mode);
}
DistanceSensor::DetectionMode DistanceSensor::getDetectionMode(void)
{
    uint mode;
    if (!readRegister(0x08, &mode))
    {
        return DETECTION_MODE_ERROR;
    }
    if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_ANY_VALID_RANGE)
    {
        return DETECTION_MODE_ERROR;
    }
    return (DetectionMode)mode;
}
bool DistanceSensor::setDetectionInverted(bool inverted)
{
    return writeRegister(0x09, inverted ? 1 : 0);
}
bool DistanceSensor::getDetectionInverted(bool &inverted)
{
    uint value;
    if (!readRegister(0x09, &value))
    {
        return false;
    }
    inverted = value != 0;
    return true;
}
bool DistanceSensor::getDetectionState(bool &detected)
{
    uint value;
    if (!readRegister(0x07, &value))
    {
        return false;
    }
    detected = value != 0;
    return true;
}
int DistanceSensor::getDetectionLowerThreshold_mm(void)
{
    uint threshold;
    if (!readRegister(0x0A, &threshold))
    {
        return -1;
    }
    return (int)threshold;
}
bool DistanceSensor::setDetectionLowerThreshold_mm(uint threshold)
{
    return writeRegister(0x0A, threshold);
}
int DistanceSensor::getDetectionUpperThreshold_mm(void)
{
    uint threshold;
    if (!readRegister(0x0B, &threshold))
    {
        return -1;
    }
    return (int)threshold;
}
bool DistanceSensor::setDetectionUpperThreshold_mm(uint threshold)
{
    return writeRegister(0x0B, threshold);
}
