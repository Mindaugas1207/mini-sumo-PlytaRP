
#ifndef IMU_H
#define IMU_H

void imu_init(vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias);
bool imu_try_get(vmath::vect_t<double>& newOrientation);
bool imu_get(vmath::vect_t<double>& newOrientation, uint timeout_ms);
void imu_calibrate(void);
vmath::vect_t<double> getAccelBias();
vmath::vect_t<double> getGyroBias();

#endif // IMU_H
