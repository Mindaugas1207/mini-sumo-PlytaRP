
#include "config.h"
#include <stdio.h>
#include <stdarg.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/mutex.h"
#include "pico/sem.h"
#include "pico/multicore.h"
#include "vmath.h"
#include "imu.h"
#include "lsm6dsr.h"
#include "system.h"
#include "async_imu.h"

#define IMU_CORE1_START_SIGNAL 123
#define IMU_CORE1_STOP_SIGNAL 321

bool running = false;
semaphore data_available_sem;
mutex data_mutex;

vmath::vect_t<double> AccelBias;
vmath::vect_t<double> GyroBias;
vmath::vect_t<double> Gyro = {0};
vmath::vect_t<double> Accel = {0};
vmath::MadgwickFilter<double> filter;
vmath::vect_t<double> Orientation = {0};

absolute_time_t filter_timestamp = 0;
absolute_time_t data_timestamp = 0;

IMU* imu_ref;

bool async_imu_init(IMU* imu, vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias, double beta)
{
    if (running) return false;
    mutex_init(&data_mutex);
    sem_init(&data_available_sem, 0, 1);

    imu_ref = imu;

    filter.Init(beta);
    filter_timestamp = get_absolute_time();
    data_timestamp = filter_timestamp;
    running = false;
    async_imu_setBias(accelBias, gyroBias);
    multicore_launch_core1(async_imu_task);
    
    return true;
}

bool async_imu_setBias(vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias)
{
    if (running) return false;
    AccelBias = accelBias;
    GyroBias = gyroBias;
    return true;
}

bool async_imu_calibrate(vmath::vect_t<double>& outAccelBias, vmath::vect_t<double>& outGyroBias)
{
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, async_imu_dataAvailableISR);

    uint calibrationSamples = 0;
    vmath::vect_t<double> gyroSum = {0};
    vmath::vect_t<double> accelSum = {0};

    absolute_time_t startTime = get_absolute_time();

    while (absolute_time_diff_us(startTime, get_absolute_time()) < (5000 * 1000))
    {
        absolute_time_t time = get_absolute_time();
        if (sem_try_acquire(&data_available_sem) || absolute_time_diff_us(data_timestamp, time) > (100 * 1000))
        {
            data_timestamp = time;
            vmath::vect_t<double> gyro;
            vmath::vect_t<double> accel;
            if (!imu_ref->readData(accel, gyro))
            {
                continue;
            }

            gyroSum += gyro;
            accelSum += accel;
            calibrationSamples++;
        }
    }

    outGyroBias = gyroSum / (double)calibrationSamples;

    accelSum /= (double)calibrationSamples;
    if(accelSum.Z > IMU_G_EARTH / 3) accelSum.Z -= IMU_G_EARTH;  // Remove gravity from the z-axis accelerometer bias calculation
    else accelSum.Z += IMU_G_EARTH;
    outAccelBias = accelSum;
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, false, nullptr);
    return true;
}
absolute_time_t print_timestamp = 0;
bool async_imu_update()
{
    if (!running) return false;
    
    if (sem_try_acquire(&data_available_sem) || absolute_time_diff_us(data_timestamp, get_absolute_time()) > (100 * 1000))
    {
        data_timestamp = get_absolute_time();

        struct LSM6DSR::Data_s raw_data;
        vmath::vect_t<double> gyro;
        vmath::vect_t<double> accel;
        if (!imu_ref->readData(accel, gyro))
        {
            return false;
        }

        Gyro = gyro - GyroBias;
        Accel = accel - AccelBias;
    }
    absolute_time_t time = get_absolute_time();
    double deltaT = ((double)absolute_time_diff_us(filter_timestamp, time)) / (1000 * 1000);
    filter_timestamp = time;
    auto quat = filter.Compute(Gyro, Accel, deltaT);
    auto angles = quat.EulerAnglesDegrees();
    /*if (absolute_time_diff_us(print_timestamp, get_absolute_time()) > (200 * 1000))
    {
        print_timestamp = get_absolute_time();
        printf("angles: X %.4f, Y %.4f, Z %.4f, accel X %.4f, Y %.4f, Z %.4f, gyro X %.4f, Y %.4f, Z %.4f\n",
            angles.X, angles.Y, angles.Z,
            Accel.X, Accel.Y, Accel.Z,
            Gyro.X, Gyro.Y, Gyro.Z);
    }*/
    if (!mutex_try_enter(&data_mutex, NULL)) return true;
    Orientation = angles;
    mutex_exit(&data_mutex);
    return true;
}

bool async_imu_tryGetData(vmath::vect_t<double>& newOrientation)
{
    if (!mutex_try_enter(&data_mutex, NULL)) return false;

    newOrientation = Orientation;

    mutex_exit(&data_mutex);

    return true;
}

void async_imu_dataAvailableISR(uint gpio, uint32_t events)
{
    sem_release(&data_available_sem);
}

void async_imu_task()
{
    while (true)
    {
        uint32_t signal = 0;
        if (multicore_fifo_pop_timeout_us(0, &signal))
        {
            if (signal == IMU_CORE1_STOP_SIGNAL)
            {
                running = false;
                gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, false, nullptr);
                multicore_fifo_push_blocking(IMU_CORE1_STOP_SIGNAL);
            }
            else if (signal == IMU_CORE1_START_SIGNAL)
            {
                running = true;
                gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, async_imu_dataAvailableISR);
                filter.Reset();
                filter_timestamp = get_absolute_time();
                multicore_fifo_push_blocking(IMU_CORE1_START_SIGNAL);
            }
        }

        async_imu_update();
    }

    //gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, false, nullptr);
    //multicore_fifo_push_blocking(IMU_CORE1_STOP_SIGNAL);
}

bool async_imu_start()
{
    uint32_t signal = 0;
    multicore_fifo_push_blocking(IMU_CORE1_START_SIGNAL);
    printf("Starting async IMU...\n");
    if (!multicore_fifo_pop_timeout_us(500, &signal))
    {
        return false;
    }
    if (signal != IMU_CORE1_START_SIGNAL)
    {
        return false;
    }
    return true;
}

bool async_imu_stop()
{
    uint32_t signal = 0;
    multicore_fifo_push_blocking(IMU_CORE1_STOP_SIGNAL);
    printf("Stopping IMU...\n");
    if (!multicore_fifo_pop_timeout_us(500, &signal))
    {
        return false;
    }
    if (signal != IMU_CORE1_STOP_SIGNAL)
    {
        return false;
    }
    return true;
}
