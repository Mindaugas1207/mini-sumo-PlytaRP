
#ifndef IMU_H
#define IMU_H

#include "vmath.h"

class IMU
{
public:
    virtual bool readData(vmath::vect_t<double>& accel, vmath::vect_t<double>& gyro){return false;};
};

#endif // IMU_H
