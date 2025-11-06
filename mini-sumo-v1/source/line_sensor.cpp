
#include <cstdlib>
#include "pico/stdlib.h"
#include "line_sensor.h"

#define LINE_VALUE_AVG_TOTAL_REG (0x01)
#define LINE_VALUE_AVG0_REG (0x02)
#define LINE_VALUE_AVG1_REG (0x03)
#define LINE_VALUE_RAW0_REG (0x04)
#define LINE_VALUE_RAW1_REG (0x05)
#define LINE_DET_OUT_REG (0x07)

#define LINE_CFG_FSMP_REG (0xB0)
#define LINE_CFG_NAVG_REG (0xB1)

#define LINE_CFG_MAX_REG (0xB7)
#define LINE_CFG_MIN_REG (0xB8)

#define LINE_CFG_DET_MODE_REG (0xBA)
#define LINE_CFG_DET_INV_REG (0xBB)
#define LINE_CFG_DET_LOW_REG (0xBC)
#define LINE_CFG_DET_HIGH_REG (0xBD)

LineSensor::LineSensor(OneWire &oneWire, uint index) : OneWireDevice(oneWire, index)
{
}

int LineSensor::getTotalAverage(void)
{
    uint value;
    if (!readRegister(LINE_VALUE_AVG_TOTAL_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getS0Average(void)
{
    uint value;
    if (!readRegister(LINE_VALUE_AVG0_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getS1Average(void)
{
    uint value;
    if (!readRegister(LINE_VALUE_AVG1_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getS0Raw(void)
{
    uint value;
    if (!readRegister(LINE_VALUE_RAW0_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getS1Raw(void)
{
    uint value;
    if (!readRegister(LINE_VALUE_RAW1_REG, &value))
    {
        return -1;
    }
    return (int)value;
}

int LineSensor::getFSMP(void)
{
    uint value;
    if (!readRegister(LINE_CFG_FSMP_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getNAVG(void)
{
    uint value;
    if (!readRegister(LINE_CFG_NAVG_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getMax(void)
{
    uint value;
    if (!readRegister(LINE_CFG_MAX_REG, &value))
    {
        return -1;
    }
    return (int)value;
}
int LineSensor::getMin(void)
{
    uint value;
    if (!readRegister(LINE_CFG_MIN_REG, &value))
    {
        return -1;
    }
    return (int)value;
}

bool LineSensor::setFSMP(uint fsmp)
{
    return writeRegister(LINE_CFG_FSMP_REG, fsmp);
}
bool LineSensor::setNAVG(uint navg)
{
    return writeRegister(LINE_CFG_NAVG_REG, navg);
}
bool LineSensor::setMax(uint max)
{
    return writeRegister(LINE_CFG_MAX_REG, max);
}
bool LineSensor::setMin(uint min)
{
    return writeRegister(LINE_CFG_MIN_REG, min);
}

bool LineSensor::setDetectionMode(DetectionMode mode)
{
    if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_THRESHOLD)
    {
        return false;
    }
    return writeRegister(LINE_CFG_DET_MODE_REG, mode);
}
LineSensor::DetectionMode LineSensor::getDetectionMode(void)
{
    uint mode;
    if (!readRegister(LINE_CFG_DET_MODE_REG, &mode))
    {
        return DETECTION_MODE_ERROR;
    }
    if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_THRESHOLD)
    {
        return DETECTION_MODE_ERROR;
    }
    return (DetectionMode)mode;
}
bool LineSensor::setDetectionInverted(bool inverted)
{
    return writeRegister(LINE_CFG_DET_INV_REG, inverted ? 1 : 0);
}
bool LineSensor::getDetectionInverted(bool &inverted)
{
    uint value;
    if (!readRegister(LINE_CFG_DET_INV_REG, &value))
    {
        return false;
    }
    inverted = value != 0;
    return true;
}
bool LineSensor::getDetectionState(bool &detected)
{
    uint value;
    if (!readRegister(LINE_DET_OUT_REG, &value))
    {
        return false;
    }
    detected = value != 0;
    return true;
}
int LineSensor::getDetectionLowerThreshold(void)
{
    uint threshold;
    if (!readRegister(LINE_CFG_DET_LOW_REG, &threshold))
    {
        return -1;
    }
    return (int)threshold;
}
bool LineSensor::setDetectionLowerThreshold(uint threshold)
{
    return writeRegister(LINE_CFG_DET_LOW_REG, threshold);
}
int LineSensor::getDetectionUpperThreshold(void)
{
    uint threshold;
    if (!readRegister(LINE_CFG_DET_HIGH_REG, &threshold))
    {
        return -1;
    }
    return (int)threshold;
}
bool LineSensor::setDetectionUpperThreshold(uint threshold)
{
    return writeRegister(LINE_CFG_DET_HIGH_REG, threshold);
}
