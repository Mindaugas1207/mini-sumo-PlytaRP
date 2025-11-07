
#ifndef ASYNC_IMU_H
#define ASYNC_IMU_H

#define IMU_G_EARTH 1.0

bool async_imu_init(IMU* imu, vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias, double beta);
bool async_imu_setBias(vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias);
bool async_imu_calibrate(vmath::vect_t<double>& outAccelBias, vmath::vect_t<double>& outGyroBias);
bool async_imu_update();
bool async_imu_tryGetData(vmath::vect_t<double>& newOrientation);
void async_imu_dataAvailableISR(uint gpio, uint32_t events);
void async_imu_task();
bool async_imu_start();
bool async_imu_stop();

#endif // ASYNC_IMU_H
