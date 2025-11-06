
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
#include "madgwickFilter.h"

#define LSM6DSR_I2C I2C_PORT

#define LSM6DSR_ADDRESS (0x6A)

#define INT1_CTRL (0x0D)
#define INT1_DRDY_XL_bit (0x01)
#define INT1_DRDY_G_bit (0x02)

#define CTRL1_XL (0x10)
#define CTRL2_G (0x11)
#define CTRL3_C (0x12)
#define BDU_bit (0x40)

#define CTRL4_C (0x13)
#define DRDY_MASK_bit (0x08)

#define STATUS_REG (0x1E)
#define OUT_TEMP_L (0x20)
#define OUT_TEMP_H (0x21)

#define OUTX_L_G (0x22)
#define OUTX_H_G (0x23)
#define OUTY_L_G (0x24)
#define OUTY_H_G (0x25)
#define OUTZ_L_G (0x26)
#define OUTZ_H_G (0x27)

#define OUTX_L_A (0x28)
#define OUTX_H_A (0x29)
#define OUTY_L_A (0x2A)
#define OUTY_H_A (0x2B)
#define OUTZ_L_A (0x2C)
#define OUTZ_H_A (0x2D)

#define ACEL_SCALE (16)
// g
#define GYRO_SCALE (4000)
// DPS

#define ACCEL_TO_G(raw) (raw * 0.488)
#define GYRO_TO_DPS(raw) (raw * ((140.0 * M_PI) / (2*180.0 * 1000.0)))

struct LSM6DSR_Data_s
{
    vmath::vect_t<double> accelerometer;
    vmath::vect_t<double> gyroscope;
};

semaphore imu_sem;
mutex heading_mutex;
volatile vmath::vect_t<double> Orientation;
vmath::vect_t<double> AccelBias;
vmath::vect_t<double> GyroBias;

bool LSM6DSR_init(void);
bool LSM6DSR_read(struct LSM6DSR_Data_s &result);
bool LSM6DSR_read(uint8_t reg, uint8_t *buffer, size_t count);
bool LSM6DSR_read(uint8_t reg, uint8_t &value);
bool LSM6DSR_write(uint8_t reg, uint8_t *buffer, size_t count);
bool LSM6DSR_write(uint8_t reg, uint8_t value);
void imu_core1_task();

void data_update(vmath::vect_t<double>& newOrientation)
{
    if (!mutex_try_enter(&heading_mutex, NULL)) return;

    Orientation.X = newOrientation.X;
    Orientation.Y = newOrientation.Y;
    Orientation.Z = newOrientation.Z;

    mutex_exit(&heading_mutex);
}

vmath::vect_t<double> getGyroBias()
{
    mutex_enter_blocking(&heading_mutex);
    auto tmp = GyroBias;
    mutex_exit(&heading_mutex);
    return tmp;
}

vmath::vect_t<double> getAccelBias()
{
    mutex_enter_blocking(&heading_mutex);
    auto tmp = AccelBias;
    mutex_exit(&heading_mutex);
    return tmp;
}

void setGyroBias(vmath::vect_t<double> bias)
{
    mutex_enter_blocking(&heading_mutex);
    GyroBias = bias;
    mutex_exit(&heading_mutex);
}

void setAccelBias(vmath::vect_t<double> bias)
{
    mutex_enter_blocking(&heading_mutex);
    AccelBias = bias;
    mutex_exit(&heading_mutex);
}


void imu_init(vmath::vect_t<double> accelBias, vmath::vect_t<double> gyroBias)
{
    AccelBias = accelBias;
    GyroBias = gyroBias;
    mutex_init(&heading_mutex);
    multicore_launch_core1(imu_core1_task);
}

bool imu_try_get(vmath::vect_t<double>& newOrientation)
{
    if (!mutex_try_enter(&heading_mutex, NULL)) return false;

    newOrientation.X = Orientation.X;
    newOrientation.Y = Orientation.Y;
    newOrientation.Z = Orientation.Z;

    mutex_exit(&heading_mutex);

    return true;
}

bool imu_get(vmath::vect_t<double>& newOrientation, uint timeout_ms)
{
    if (!mutex_enter_timeout_ms(&heading_mutex, timeout_ms)) return false;

    newOrientation.X = Orientation.X;
    newOrientation.Y = Orientation.Y;
    newOrientation.Z = Orientation.Z;

    mutex_exit(&heading_mutex);

    return true;
}

void imu_calibrate(void)
{
    multicore_fifo_push_blocking(123);
    uint32_t fifo_data = 0;

    if (multicore_fifo_pop_blocking() != 1) error_handler("Failed to start imu calibration\n");
    if (multicore_fifo_pop_blocking() != 123) error_handler("Failed to end imu calibration\n");
}

#define IMU_HW_G_EARTH 9.81492

void imu_core1_task()
{
    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, false);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, [](uint gpio, uint32_t events){sem_release(&imu_sem);});

    sem_init(&imu_sem, 0, 1);

    while (!LSM6DSR_init())
    {
        sleep_ms(50);
    }

    absolute_time_t timestamp = get_absolute_time();
    struct LSM6DSR_Data_s raw_data = {};

    vmath::vect_t<double> _AccelBias = getAccelBias();
    vmath::vect_t<double> _GyroBias = getGyroBias();
    uint calibrationSamples = 0;


    vmath::MadgwickFilter<double> Filter;
    absolute_time_t filter_timestamp = get_absolute_time();
    bool calibrated = true;
    vmath::vect_t<double> _Orientation = {};
    
    Filter.Init(0.01);

    while (true)
    {
        uint32_t fifo_data = 0;
        if (multicore_fifo_pop_timeout_us(0, &fifo_data))
        {
            if (fifo_data == 123)
            {   
                multicore_fifo_push_blocking(1);
                filter_timestamp = get_absolute_time();
                _AccelBias = 0;
                _GyroBias = 0;
                calibrationSamples = 0;
                calibrated = false;
            }
        }

        if (sem_try_acquire(&imu_sem) || absolute_time_diff_us(timestamp, get_absolute_time()) > (100 * 1000))
        {
            timestamp = get_absolute_time();

            if (!LSM6DSR_read(raw_data))
            {
                continue;
            }

            if (!calibrated)
            {
                if (absolute_time_diff_us(filter_timestamp, get_absolute_time()) > (5000 * 1000))
                {
                    filter_timestamp = get_absolute_time();
                    _GyroBias  = _GyroBias  / calibrationSamples;
                    _AccelBias = _AccelBias / calibrationSamples;

                    if(_AccelBias.Z > IMU_HW_G_EARTH / 3) _AccelBias.Z -= IMU_HW_G_EARTH;  // Remove gravity from the z-axis accelerometer bias calculation
                    else _AccelBias.Z += IMU_HW_G_EARTH;

                    setGyroBias(_GyroBias);
                    setAccelBias(_AccelBias);

                    calibrated = true;
                    multicore_fifo_push_blocking(123);
                }
                else
                {
                    _AccelBias += raw_data.accelerometer;
                    _GyroBias += raw_data.gyroscope;
                    calibrationSamples++;
                }
            }
        }

        if (calibrated)
        {
            absolute_time_t time = get_absolute_time();
            double deltaT = ((double)absolute_time_diff_us(filter_timestamp, time)) / (1000 * 1000);
            filter_timestamp = time;
            auto gyro = raw_data.gyroscope - _GyroBias;
            auto accel = raw_data.accelerometer - _AccelBias;

            imu_filter(accel.X, accel.Y, accel.Z, gyro.X, gyro.Y, gyro.Z, deltaT);
            double roll, pitch, yaw;
            eulerAngles(q_est, &roll, &pitch, &yaw);

            //auto quat = Filter.Compute(gyro, accel, deltaT);
            //auto angles = quat.EulerAngles();
            //_Orientation = angles * 0.9 + _Orientation * 0.1;
            _Orientation.Roll(roll);
            _Orientation.Pitch(pitch);
            _Orientation.Yaw(yaw);
            data_update(_Orientation);
        }
        

        

        

        

        

        

        //Orientation = angles * 0.9 + Orientation * 0.1;
    }
}

bool LSM6DSR_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[] = {reg, value};
    return i2c_write_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, buf, sizeof(buf), false) != PICO_ERROR_GENERIC;
}

bool LSM6DSR_write(uint8_t reg, uint8_t *buffer, size_t count)
{
    bool error;
    error = i2c_write_burst_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, &reg, 1) != PICO_ERROR_GENERIC;
    error |= i2c_write_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, buffer, count, false) == PICO_ERROR_GENERIC;
    return !error;
}

bool LSM6DSR_read(uint8_t reg, uint8_t &value)
{
    bool error;
    error = i2c_write_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, &reg, 1, true) == PICO_ERROR_GENERIC;
    error |= i2c_read_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, &value, 1, false) == PICO_ERROR_GENERIC;
    return !error;
}

bool LSM6DSR_read(uint8_t reg, uint8_t *buffer, size_t count)
{
    bool error;
    error = i2c_write_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, &reg, 1, true) == PICO_ERROR_GENERIC;
    error |= i2c_read_blocking(LSM6DSR_I2C, LSM6DSR_ADDRESS, buffer, count, false) == PICO_ERROR_GENERIC;
    return !error;
}

bool LSM6DSR_read(struct LSM6DSR_Data_s &result)
{
    constexpr int offset = STATUS_REG;
    uint8_t buffer[OUTZ_H_A - STATUS_REG + 1];
    if (!LSM6DSR_read(STATUS_REG, buffer, sizeof(buffer)))
    {
        return false;
    }

    result.accelerometer.X = ACCEL_TO_G((int16_t)((buffer[OUTX_H_A - offset] << 8) | buffer[OUTX_L_A - offset]));
    result.accelerometer.Y = ACCEL_TO_G((int16_t)((buffer[OUTY_H_A - offset] << 8) | buffer[OUTY_L_A - offset]));
    result.accelerometer.Z = ACCEL_TO_G((int16_t)((buffer[OUTZ_H_A - offset] << 8) | buffer[OUTZ_L_A - offset]));
    result.gyroscope.X = GYRO_TO_DPS((int16_t)((buffer[OUTX_H_G - offset] << 8) | buffer[OUTX_L_G - offset]));
    result.gyroscope.Y = GYRO_TO_DPS((int16_t)((buffer[OUTY_H_G - offset] << 8) | buffer[OUTY_L_G - offset]));
    result.gyroscope.Z = GYRO_TO_DPS((int16_t)((buffer[OUTZ_H_G - offset] << 8) | buffer[OUTZ_L_G - offset]));

    return true;
}

bool LSM6DSR_init(void)
{
    uint8_t stat = 0;
    if(!LSM6DSR_read(0x16, stat)) return false;
    if(!LSM6DSR_write(INT1_CTRL, INT1_DRDY_G_bit)) return false;
    //6 = 416HZ
    //7 = 833HZ
    //8 = 1.66KHZ
    //9 = 3.33KHZ
    if(!LSM6DSR_write(CTRL1_XL, 0x94)) return false;
    if(!LSM6DSR_write(CTRL2_G, 0x9C)) return false;
    return true;
}
