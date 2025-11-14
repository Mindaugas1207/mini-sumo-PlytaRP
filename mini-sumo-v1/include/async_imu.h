
#ifndef ASYNC_IMU_H
#define ASYNC_IMU_H

#define IMU_G_EARTH 1.0

bool async_imu_init(IMU *imu, vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias, double beta);
bool async_imu_getBias(vmath::vect_t<double> &outAccelBias, vmath::vect_t<double> &outGyroBias);
bool async_imu_tryGetOrientation(vmath::vect_t<double> &newOrientation);
bool async_imu_start(void);
bool async_imu_stop(void);
bool async_imu_reset(void);
bool async_imu_calibrate(void);

#endif // ASYNC_IMU_H
