
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "LSM6DSR.h"

LSM6DSR::LSM6DSR(i2c_inst_t* i2c, int timeout_ms) : i2c(i2c), timeout_ms(timeout_ms)
{
}

bool LSM6DSR::begin(AccelerometerRange accelRange, GyroscopeRange gyroRange, DataRate accelRate, DataRate gyroRate)
{
    uint8_t whoami = 0;
    absolute_time_t start = get_absolute_time();

    while (!whoAmI(whoami))
    {
        if (absolute_time_diff_us(start, get_absolute_time()) > LSM6DSR_INIT_DELAY_MS * 1000)
        {
            return false;
        }
        sleep_ms(10);
    }

    if (!reset())
    {
        return false;
    }

    start = get_absolute_time();
    while (!whoAmI(whoami))
    {
        if (absolute_time_diff_us(start, get_absolute_time()) > LSM6DSR_INIT_DELAY_MS * 1000)
        {
            return false;
        }
        sleep_ms(10);
    }

    if (whoami != LSM6DSR_WHO_AM_I_RESPONSE)
    {
        return false;
    }

    if (!setAccelerometerRange(accelRange))
    {
        return false;
    }

    if (!setGyroscopeRange(gyroRange))
    {
        return false;
    }

    if (!setDataRate(accelRate, gyroRate))
    {
        return false;
    }

    return true;
}

bool LSM6DSR::reset(int timeout_ms)
{
    uint8_t ctrl1xl = 0;
    uint8_t ctrl2g = 0;
    bool complete = false;
    absolute_time_t start = get_absolute_time();
    while (!complete)
    {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_ms * 1000)
        {
            return false;
        }

        if (!read(LSM6DSR_CTRL1_XL, ctrl1xl) || !read(LSM6DSR_CTRL2_G, ctrl2g))
        {
            continue;
        }

        if (((ctrl1xl & 0xF0) == 0) && ((ctrl2g & 0xF0) == 0))
        {
            complete = true;
        }
        else
        {
            write(LSM6DSR_RESET_REG, LSM6DSR_RESET_BIT);
            sleep_ms(LSM6DSR_INIT_DELAY_MS);
        }
    }

    return true;
}

bool LSM6DSR::setAccelerometerRange(AccelerometerRange range)
{
    uint8_t ctrl1xl = 0;
    if (!read(LSM6DSR_CTRL1_XL, ctrl1xl))
    {
        return false;
    }

    ctrl1xl &= ~0x0C; // Clear FS bits
    ctrl1xl |= range;

    if (!write(LSM6DSR_CTRL1_XL, ctrl1xl))
    {
        return false;
    }

    switch (range)
    {
    case ACCEL_RANGE_2G:
        accelScaleFactor = LSM6DSR_ACCEL_FS2G_G_PER_LSB;
        break;
    case ACCEL_RANGE_4G:
        accelScaleFactor = LSM6DSR_ACCEL_FS4G_G_PER_LSB;
        break;
    case ACCEL_RANGE_8G:
        accelScaleFactor = LSM6DSR_ACCEL_FS8G_G_PER_LSB;
        break;
    case ACCEL_RANGE_16G:
        accelScaleFactor = LSM6DSR_ACCEL_FS16G_G_PER_LSB;
        break;
    default:
        accelScaleFactor = 1.0;
        break;
    }

    return true;
}

bool LSM6DSR::setGyroscopeRange(GyroscopeRange range)
{
    uint8_t ctrl2g = 0;
    if (!read(LSM6DSR_CTRL2_G, ctrl2g))
    {
        return false;
    }

    ctrl2g &= ~0x0F; // Clear FS bits
    ctrl2g |= range;

    if (!write(LSM6DSR_CTRL2_G, ctrl2g))
    {
        return false;
    }

    switch (range)
    {
    case GYRO_RANGE_125DPS:
        gyroScaleFactor = LSM6DSR_GYRO_FS125_DPS_PER_LSB;
        break;
    case GYRO_RANGE_250DPS:
        gyroScaleFactor = LSM6DSR_GYRO_FS250_DPS_PER_LSB;
        break;
    case GYRO_RANGE_500DPS:
        gyroScaleFactor = LSM6DSR_GYRO_FS500_DPS_PER_LSB;
        break;
    case GYRO_RANGE_1000DPS:
        gyroScaleFactor = LSM6DSR_GYRO_FS1000_DPS_PER_LSB;
        break;
    case GYRO_RANGE_2000DPS:
        gyroScaleFactor = LSM6DSR_GYRO_FS2000_DPS_PER_LSB;
        break;
    case GYRO_RANGE_4000DPS:
        gyroScaleFactor = LSM6DSR_GYRO_FS4000_DPS_PER_LSB;
        break;
    default:
        gyroScaleFactor = 1.0;
        break;
    }

    return true;
}

bool LSM6DSR::setDataRate(DataRate accelRate, DataRate gyroRate)
{
    uint8_t ctrl1xl = 0;
    if (!read(LSM6DSR_CTRL1_XL, ctrl1xl))
    {
        return false;
    }

    ctrl1xl &= ~0xF0; // Clear ODR bits
    ctrl1xl |= accelRate;

    if (!write(LSM6DSR_CTRL1_XL, ctrl1xl))
    {
        return false;
    }

    uint8_t ctrl2g = 0;
    if (!read(LSM6DSR_CTRL2_G, ctrl2g))
    {
        return false;
    }

    ctrl2g &= ~0xF0; // Clear ODR bits
    ctrl2g |= gyroRate;

    return write(LSM6DSR_CTRL2_G, ctrl2g);
}

bool LSM6DSR::setInterrupts(bool accelDataReadyEnable, bool gyroDataReadyEnable)
{
    uint8_t int1Ctrl = 0;
    if (!read(LSM6DSR_INT1_CTRL, int1Ctrl))
    {
        return false;
    }

    int1Ctrl &= ~(LSM6DSR_INT1_DRDY_XL | LSM6DSR_INT1_DRDY_G); // Clear data ready bits

    if (accelDataReadyEnable)
    {
        int1Ctrl |= LSM6DSR_INT1_DRDY_XL;
    }

    if (gyroDataReadyEnable)
    {
        int1Ctrl |= LSM6DSR_INT1_DRDY_G;
    }

    return write(LSM6DSR_INT1_CTRL, int1Ctrl);
}

bool LSM6DSR::readData(struct Data_s &result)
{
    constexpr int offset = LSM6DSR_STATUS_REG;
    uint8_t buffer[LSM6DSR_OUTZ_H_A - LSM6DSR_STATUS_REG + 1];
    if (!read(LSM6DSR_STATUS_REG, buffer, sizeof(buffer)))
    {
        return false;
    }

    result.status = buffer[0];
    result.temperature = LSM6DSR_TEMP_DEG_C_PER_LSB * ((int16_t)((buffer[LSM6DSR_OUT_TEMP_H - offset] << 8) | buffer[LSM6DSR_OUT_TEMP_L - offset])) + LSM6DSR_TEMP_OFFSET_DEG_C;
    result.gyroscope.X = gyroScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTX_H_G - offset] << 8) | buffer[LSM6DSR_OUTX_L_G - offset]));
    result.gyroscope.Y = gyroScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTY_H_G - offset] << 8) | buffer[LSM6DSR_OUTY_L_G - offset]));
    result.gyroscope.Z = gyroScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTZ_H_G - offset] << 8) | buffer[LSM6DSR_OUTZ_L_G - offset]));
    result.accelerometer.X = accelScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTX_H_A - offset] << 8) | buffer[LSM6DSR_OUTX_L_A - offset]));
    result.accelerometer.Y = accelScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTY_H_A - offset] << 8) | buffer[LSM6DSR_OUTY_L_A - offset]));
    result.accelerometer.Z = accelScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTZ_H_A - offset] << 8) | buffer[LSM6DSR_OUTZ_L_A - offset]));

    return true;
}

bool LSM6DSR::readData(vmath::vect_t<double>& accel, vmath::vect_t<double>& gyro)
{
    struct Data_s raw_data;
    if (!readData(raw_data))
    {
        return false;
    }

    accel.X = raw_data.accelerometer.X;
    accel.Y = raw_data.accelerometer.Y;
    accel.Z = raw_data.accelerometer.Z;

    gyro.X = raw_data.gyroscope.X * (M_PI / 180.0);
    gyro.Y = raw_data.gyroscope.Y * (M_PI / 180.0);
    gyro.Z = raw_data.gyroscope.Z * (M_PI / 180.0);

    return true;
}

bool LSM6DSR::whoAmI(uint8_t &whoami)
{
    return read(LSM6DSR_WHO_AM_I_REG, whoami);
}

bool LSM6DSR::write(uint8_t reg, uint8_t value)
{
    uint8_t buf[] = {reg, value};
    bool error = i2c_write_blocking_until(i2c, LSM6DSR_ADDRESS, buf, sizeof(buf), false, make_timeout_time_ms(timeout_ms)) != 2;
    return !error;
}

bool LSM6DSR::read(uint8_t reg, uint8_t &value)
{
    bool error;
    error = i2c_write_blocking_until(i2c, LSM6DSR_ADDRESS, &reg, 1, true, make_timeout_time_ms(timeout_ms)) != 1;
    error |= i2c_read_blocking_until(i2c, LSM6DSR_ADDRESS, &value, 1, false, make_timeout_time_ms(timeout_ms)) != 1;
    return !error;
}

bool LSM6DSR::read(uint8_t reg, uint8_t *buffer, size_t count)
{
    bool error;
    error = i2c_write_blocking_until(i2c, LSM6DSR_ADDRESS, &reg, 1, true, make_timeout_time_ms(timeout_ms)) != 1;
    error |= i2c_read_blocking_until(i2c, LSM6DSR_ADDRESS, buffer, count, false, make_timeout_time_ms(timeout_ms)) != count;
    return !error;
}
