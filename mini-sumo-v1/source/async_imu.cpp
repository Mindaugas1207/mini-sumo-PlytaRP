
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
#include "config.h"

#define ASYNC_IMU_OK_SIGNAL 0
#define ASYNC_IMU_FAIL_SIGNAL -1
#define ASYNC_IMU_START_SIGNAL 123
#define ASYNC_IMU_STOP_SIGNAL 321
#define ASYNC_IMU_RESET_SIGNAL 444
#define ASYNC_IMU_CALIBRATE_SIGNAL 696

#define ASYNC_IMU_INT_TIMEOUT (100 * 1000)
#define ASYNC_IMU_CALIBRATION_DURATION (5000 * 1000)

bool running = false;
bool reset = false;
semaphore data_available_sem;
mutex data_mutex;

vmath::vect_t<double> AccelBias;
vmath::vect_t<double> GyroBias;
vmath::MadgwickFilter<double> filter;
vmath::vect_t<double> Orientation = {0};

IMU *imu_ref;

void update(void);
void calibrate(vmath::vect_t<double> &outAccelBias, vmath::vect_t<double> &outGyroBias);
bool readData(vmath::vect_t<double>& accel, vmath::vect_t<double>& gyro);
void asyncTask(void);

bool async_imu_init(IMU *imu, vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias, double beta)
{
    if (running)
        return false;
    mutex_init(&data_mutex);
    sem_init(&data_available_sem, 0, 1);

    imu_ref = imu;

    filter.Init(beta);
    running = false;
    reset = true;
    AccelBias = accelBias;
    GyroBias = gyroBias;
    multicore_launch_core1(asyncTask);

    return true;
}

bool async_imu_getBias(vmath::vect_t<double> &outAccelBias, vmath::vect_t<double> &outGyroBias)
{
    if (running)
        return false;
    outAccelBias = AccelBias;
    outGyroBias = GyroBias;
    return true;
}

bool async_imu_tryGetOrientation(vmath::vect_t<double> &newOrientation)
{
    if (!mutex_try_enter(&data_mutex, NULL))
        return false;

    newOrientation = Orientation;

    mutex_exit(&data_mutex);

    return true;
}

bool fifo_pop(uint32_t& signal, uint32_t timeout)
{
    absolute_time_t timestamp = get_absolute_time();
    do
    {
        if (multicore_fifo_rvalid())
        {
            signal = multicore_fifo_pop_blocking();
            return true;
        }
    } while (absolute_time_diff_us(timestamp, get_absolute_time()) < (timeout * 1000));
    
    return false;
}

bool async_imu_start(void)
{
    uint32_t signal = 0;
    multicore_fifo_drain();
    multicore_fifo_push_blocking(ASYNC_IMU_START_SIGNAL);
    debug_printf("Starting async IMU...\n");
    if (!fifo_pop(signal, 500))
    {
        return false;
    }
    if (signal != ASYNC_IMU_OK_SIGNAL)
    {
        return false;
    }
    return true;
}

bool async_imu_stop(void)
{
    uint32_t signal = 0;
    multicore_fifo_drain();
    multicore_fifo_push_blocking(ASYNC_IMU_STOP_SIGNAL);
    debug_printf("Stopping IMU...\n");
    if (!fifo_pop(signal, 500))
    {
        return false;
    }
    if (signal != ASYNC_IMU_OK_SIGNAL)
    {
        return false;
    }
    return true;
}

bool async_imu_reset(void)
{
    uint32_t signal = 0;
    multicore_fifo_drain();
    multicore_fifo_push_blocking(ASYNC_IMU_RESET_SIGNAL);
    debug_printf("Reseting IMU...\n");
    if (!fifo_pop(signal, 500))
    {
        return false;
    }
    if (signal != ASYNC_IMU_OK_SIGNAL)
    {
        return false;
    }
    return true;
}

bool async_imu_calibrate(void)
{
    uint32_t signal = 0;
    multicore_fifo_drain();
    multicore_fifo_push_blocking(ASYNC_IMU_CALIBRATE_SIGNAL);
    debug_printf("IMU Calibration start\n");
    if (!fifo_pop(signal, 500))
    {
        return false;
    }
    if (signal != ASYNC_IMU_OK_SIGNAL)
    {
        return false;
    }
    if (!fifo_pop(signal, ASYNC_IMU_CALIBRATION_DURATION * 2))
    {
        return false;
    }
    if (signal != ASYNC_IMU_OK_SIGNAL)
    {
        return false;
    }
    debug_printf("IMU Calibration end\n");
    return true;
}

uint32_t __not_in_flash_func(get_command)()
{
    if (multicore_fifo_rvalid())
    {
        return multicore_fifo_pop_blocking_inline();
    }

    return 0;
}

void __not_in_flash_func(asyncTask)()
{
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, [](uint, uint32_t){ sem_release(&data_available_sem); });

    while (true)
    {
        if (running)
        {
            uint32_t cmd = get_command();
            
            switch (cmd)
            {
            case ASYNC_IMU_STOP_SIGNAL:
                running = false;
                break;
            case ASYNC_IMU_START_SIGNAL:
                multicore_fifo_push_blocking_inline(ASYNC_IMU_OK_SIGNAL);
                break;
            case ASYNC_IMU_CALIBRATE_SIGNAL:
                reset = true;
                multicore_fifo_push_blocking_inline(ASYNC_IMU_OK_SIGNAL);
                calibrate(AccelBias, GyroBias);
                multicore_fifo_push_blocking_inline(ASYNC_IMU_OK_SIGNAL);
                break;
            case ASYNC_IMU_RESET_SIGNAL:
                reset = true;
                multicore_fifo_push_blocking_inline(ASYNC_IMU_OK_SIGNAL);
                break;
            default:
                break;
            }

            update();
        }
        else
        {
            uint32_t save = save_and_disable_interrupts();
            multicore_fifo_push_blocking_inline(ASYNC_IMU_OK_SIGNAL);
            while (get_command() != ASYNC_IMU_START_SIGNAL) {
                __wfe();
            }
            restore_interrupts_from_disabled(save);
            reset = true;
            running = true;
            multicore_fifo_push_blocking_inline(ASYNC_IMU_OK_SIGNAL);
        }
    }

    multicore_fifo_push_blocking_inline(ASYNC_IMU_FAIL_SIGNAL);
}

void update(void)
{
    static vmath::vect_t<double> Accel = {0}, Gyro = {0};
    static absolute_time_t timestamp = 0;
    

    vmath::vect_t<double> accel, gyro;
    if (readData(accel, gyro))
    {
        Accel = accel - AccelBias;
        Gyro = gyro - GyroBias;

        if (reset)
        {
            reset = false;
            filter.Reset();
            timestamp = get_absolute_time();
        }
    }
    else if (reset)
    {
        return;
    }

    absolute_time_t time = get_absolute_time();
    double deltaT = ((double)absolute_time_diff_us(timestamp, time)) / (1000 * 1000);
    timestamp = time;
    auto quat = filter.Compute(Gyro, Accel, deltaT);
    auto euler = quat.EulerAnglesDegrees();
    if (!mutex_try_enter(&data_mutex, NULL))
        return;
    Orientation = euler;
    mutex_exit(&data_mutex);

    /// DEBUG ///
    /*
    static absolute_time_t printTime = 0;
    if (absolute_time_diff_us(printTime, get_absolute_time()) > (200 * 1000))
    {
        printTime = get_absolute_time();
        printf("euler: X %.4f, Y %.4f, Z %.4f, accel X %.4f, Y %.4f, Z %.4f, gyro X %.4f, Y %.4f, Z %.4f\n",
            euler.X, euler.Y, euler.Z,
            Accel.X, Accel.Y, Accel.Z,
            Gyro.X, Gyro.Y, Gyro.Z);
    }
    */
}

void calibrate(vmath::vect_t<double> &outAccelBias, vmath::vect_t<double> &outGyroBias)
{
    vmath::vect_t<double> accelSum = {0}, gyroSum = {0};
    uint calibrationSamples = 0;
    absolute_time_t startTime = get_absolute_time();

    AccelBias = 0;
    GyroBias = 0;

    while (absolute_time_diff_us(startTime, get_absolute_time()) < ASYNC_IMU_CALIBRATION_DURATION)
    {
        vmath::vect_t<double> accel, gyro;

        if (readData(accel, gyro))
        {
            accelSum += accel;
            gyroSum += gyro;
            calibrationSamples++;
        }
    }

    outAccelBias = accelSum / calibrationSamples;
    outAccelBias.Z += (outAccelBias.Z > (IMU_G_EARTH / 3)) ? -IMU_G_EARTH : IMU_G_EARTH; // Remove gravity from bias
    outGyroBias = gyroSum / calibrationSamples;
}

bool readData(vmath::vect_t<double>& accel, vmath::vect_t<double>& gyro)
{
    static absolute_time_t timestamp = 0;
    if (sem_try_acquire(&data_available_sem) || absolute_time_diff_us(timestamp, get_absolute_time()) > ASYNC_IMU_INT_TIMEOUT)
    {
        timestamp = get_absolute_time();
        if (!imu_ref->readData(accel, gyro))
        {
            return false;
        }

        return true;
    }
    return false;
}
