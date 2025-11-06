
#include <cstdlib>
#include "pico/stdlib.h"
#include "distance_sensor.h"

#define RANGE_FLAG_REG (0x00)
#define RANGE_DIST_MM_REG (0x01)
#define RANGE_SIG_RATE_REG (0x02)
#define RANGE_AMB_RATE_REG (0x03)
#define RANGE_SIGMA_REG (0x04)
#define RANGE_SPADS_REG (0x05)
#define RANGE_DIST_CM_REG (0x06)
#define RANGE_DET_OUT_REG (0x07)

#define RANGE_CFG_TIMING_REG (0xB0)
#define RANGE_CFG_OFFSET_REG (0xB1)
#define RANGE_CFG_XTALK_REG (0xB2)
#define RANGE_CFG_LINCOR_REG (0xB3)
#define RANGE_CFG_CHECKS_REG (0xB4)
#define RANGE_CFG_SIGNAL_REG (0xB5)
#define RANGE_CFG_SIGMA_REG (0xB6)
#define RANGE_CFG_MAX_REG (0xB7)
#define RANGE_CFG_MIN_REG (0xB8)

#define RANGE_CFG_DET_MODE_REG (0xBA)
#define RANGE_CFG_DET_INV_REG (0xBB)
#define RANGE_CFG_DET_LOW_REG (0xBC)
#define RANGE_CFG_DET_HIGH_REG (0xBD)

DistanceSensor::DistanceSensor(OneWire &oneWire, uint index) : OneWireDevice(oneWire, index)
{
}
DistanceSensor::RangeFlags DistanceSensor::getRangeFlags(void)
{
    uint flags;
    if (!readRegister(RANGE_FLAG_REG, &flags))
    {
        return (RangeFlags)0;
    }
    return (RangeFlags)flags;
}
int DistanceSensor::getDistance_mm(void)
{
    uint distance;
    if (!readRegister(RANGE_DIST_MM_REG, &distance))
    {
        return -1;
    }
    return (int)distance;
}
int DistanceSensor::getDistance_cm(void)
{
    uint distance;
    if (!readRegister(RANGE_DIST_CM_REG, &distance))
    {
        return -1;
    }
    return (int)distance;
}
int DistanceSensor::getSPADCount(void)
{
    uint spadCount;
    if (!readRegister(RANGE_SPADS_REG, &spadCount))
    {
        return -1;
    }
    return (int)spadCount;
}
int DistanceSensor::getSignalRate_MCPS(void)
{
    uint signalRate;
    if (!readRegister(RANGE_SIG_RATE_REG, &signalRate))
    {
        return -1;
    }
    return (int)signalRate;
}
int DistanceSensor::getAmbientRate_MCPS(void)
{
    uint ambientRate;
    if (!readRegister(RANGE_AMB_RATE_REG, &ambientRate))
    {
        return -1;
    }
    return (int)ambientRate;
}
int DistanceSensor::getSignalSigma_mm(void)
{
    uint signalSigma;
    if (!readRegister(RANGE_SIGMA_REG, &signalSigma))
    {
        return -1;
    }
    return (int)signalSigma;
}
DistanceSensor::RangeFlags DistanceSensor::getRangeChecks(void)
{
    uint flags;
    if (!readRegister(RANGE_CFG_CHECKS_REG, &flags))
    {
        return (RangeFlags)0;
    }
    return (RangeFlags)flags;
}
int DistanceSensor::getRangeMax_mm(void)
{
    uint maxDistance;
    if (!readRegister(RANGE_CFG_MAX_REG, &maxDistance))
    {
        return -1;
    }
    return (int)maxDistance;
}
int DistanceSensor::getRangeMin_mm(void)
{
    uint minDistance;
    if (!readRegister(RANGE_CFG_MIN_REG, &minDistance))
    {
        return -1;
    }
    return (int)minDistance;
}
int DistanceSensor::getRangeTiming_ms(void)
{
    uint timing;
    if (!readRegister(RANGE_CFG_TIMING_REG, &timing))
    {
        return -1;
    }
    return (int)timing;
}
int DistanceSensor::getRangeLinearCorrection(void)
{
    uint correction;
    if (!readRegister(RANGE_CFG_LINCOR_REG, &correction))
    {
        return -1;
    }
    return (int)correction;
}
int DistanceSensor::getRangeOffset(void)
{
    uint offset;
    if (!readRegister(RANGE_CFG_OFFSET_REG, &offset))
    {
        return -1;
    }
    return (int)offset;
}
int DistanceSensor::getRangeXTALK(void)
{
    uint xtalk;
    if (!readRegister(RANGE_CFG_XTALK_REG, &xtalk))
    {
        return -1;
    }
    return (int)xtalk;
}
int DistanceSensor::getRangeSignalLimit_MCPS(void)
{
    uint limit;
    if (!readRegister(RANGE_CFG_SIGNAL_REG, &limit))
    {
        return -1;
    }
    return (int)limit;
}
int DistanceSensor::getRangeSigmaLimit_mm(void)
{
    uint limit;
    if (!readRegister(RANGE_CFG_SIGMA_REG, &limit))
    {
        return -1;
    }
    return (int)limit;
}
bool DistanceSensor::setRangeChecks(DistanceSensor::RangeFlags flags)
{
    return writeRegister(RANGE_CFG_CHECKS_REG, flags);
}
bool DistanceSensor::setRangeMax_mm(uint maxDistance)
{
    return writeRegister(RANGE_CFG_MAX_REG, maxDistance);
}
bool DistanceSensor::setRangeMin_mm(uint minDistance)
{
    return writeRegister(RANGE_CFG_MIN_REG, minDistance);
}
bool DistanceSensor::setRangeTiming_ms(uint timing)
{
    return writeRegister(RANGE_CFG_TIMING_REG, timing);
}
bool DistanceSensor::setRangeLinearCorrection(uint correction)
{
    return writeRegister(RANGE_CFG_LINCOR_REG, correction);
}
bool DistanceSensor::setRangeXTALK(uint xtalk)
{
    return writeRegister(RANGE_CFG_XTALK_REG, xtalk);
}
bool DistanceSensor::setRangeOffset(uint offset)
{
    return writeRegister(RANGE_CFG_OFFSET_REG, offset);
}
bool DistanceSensor::setRangeSignalLimit_MCPS(uint limit)
{
    return writeRegister(RANGE_CFG_SIGNAL_REG, limit);
}
bool DistanceSensor::setRangeSigmaLimit_mm(uint limit)
{
    return writeRegister(RANGE_CFG_SIGMA_REG, limit);
}
bool DistanceSensor::setDetectionMode(DistanceSensor::DetectionMode mode)
{
    if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_ANY_VALID_RANGE)
    {
        return false;
    }
    return writeRegister(RANGE_CFG_DET_MODE_REG, mode);
}
DistanceSensor::DetectionMode DistanceSensor::getDetectionMode(void)
{
    uint mode;
    if (!readRegister(RANGE_CFG_DET_MODE_REG, &mode))
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
    return writeRegister(RANGE_CFG_DET_INV_REG, inverted ? 1 : 0);
}
bool DistanceSensor::getDetectionInverted(bool &inverted)
{
    uint value;
    if (!readRegister(RANGE_CFG_DET_INV_REG, &value))
    {
        return false;
    }
    inverted = value != 0;
    return true;
}
bool DistanceSensor::getDetectionState(bool &detected)
{
    uint value;
    if (!readRegister(RANGE_DET_OUT_REG, &value))
    {
        return false;
    }
    detected = value != 0;
    return true;
}
int DistanceSensor::getDetectionLowerThreshold_mm(void)
{
    uint threshold;
    if (!readRegister(RANGE_CFG_DET_LOW_REG, &threshold))
    {
        return -1;
    }
    return (int)threshold;
}
bool DistanceSensor::setDetectionLowerThreshold_mm(uint threshold)
{
    return writeRegister(RANGE_CFG_DET_LOW_REG, threshold);
}
int DistanceSensor::getDetectionUpperThreshold_mm(void)
{
    uint threshold;
    if (!readRegister(RANGE_CFG_DET_HIGH_REG, &threshold))
    {
        return -1;
    }
    return (int)threshold;
}
bool DistanceSensor::setDetectionUpperThreshold_mm(uint threshold)
{
    return writeRegister(RANGE_CFG_DET_HIGH_REG, threshold);
}
