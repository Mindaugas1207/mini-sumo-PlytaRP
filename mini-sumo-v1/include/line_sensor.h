
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "one_wire_device.h"

class LineSensor : public OneWireDevice
{
public:
    enum DetectionMode
    {
        DETECTION_MODE_ERROR = -1,
        DETECTION_MODE_THRESHOLD = 0,
        DETECTION_MODE_WINDOW = 1
    };

    LineSensor(OneWire &oneWire, uint index);

    int getTotalAverage(void);
    int getS0Average(void);
    int getS1Average(void);
    int getS0Raw(void);
    int getS1Raw(void);

    int getFSMP(void);
    int getNAVG(void);
    int getMax(void);
    int getMin(void);

    bool setFSMP(uint fsmp);
    bool setNAVG(uint navg);
    bool setMax(uint max);
    bool setMin(uint min);

    bool setDetectionMode(DetectionMode mode);
    DetectionMode getDetectionMode(void);
    bool setDetectionInverted(bool inverted);
    bool getDetectionInverted(bool &inverted);
    bool getDetectionState(bool &detected);
    int getDetectionLowerThreshold(void);
    bool setDetectionLowerThreshold(uint threshold);
    int getDetectionUpperThreshold(void);
    bool setDetectionUpperThreshold(uint threshold);
};

#endif // DISTANCE_SENSOR_H
