
#ifndef LSM6DSR_H
#define LSM6DSR_H

#include "imu.h"

#define LSM6DSR_ADDRESS (0x6A)
#define LSM6DSR_INIT_DELAY_MS (100)

#define LSM6DSR_WHO_AM_I_REG (0x0F)
#define LSM6DSR_WHO_AM_I_RESPONSE (0x6B)

#define LSM6DSR_RESET_REG (0x12)
#define LSM6DSR_RESET_BIT (0x01)

#define LSM6DSR_INT1_DRDY_XL (0x01)
#define LSM6DSR_INT1_DRDY_G (0x02)

#define LSM6DSR_INT1_CTRL (0x0D)
#define LSM6DSR_CTRL1_XL (0x10)
#define LSM6DSR_CTRL2_G (0x11)
#define LSM6DSR_CTRL3_C (0x12)
#define LSM6DSR_CTRL4_C (0x13)
#define LSM6DSR_STATUS_REG (0x1E)
#define LSM6DSR_OUT_TEMP_L (0x20)
#define LSM6DSR_OUT_TEMP_H (0x21)
#define LSM6DSR_OUTX_L_G (0x22)
#define LSM6DSR_OUTX_H_G (0x23)
#define LSM6DSR_OUTY_L_G (0x24)
#define LSM6DSR_OUTY_H_G (0x25)
#define LSM6DSR_OUTZ_L_G (0x26)
#define LSM6DSR_OUTZ_H_G (0x27)
#define LSM6DSR_OUTX_L_A (0x28)
#define LSM6DSR_OUTX_H_A (0x29)
#define LSM6DSR_OUTY_L_A (0x2A)
#define LSM6DSR_OUTY_H_A (0x2B)
#define LSM6DSR_OUTZ_L_A (0x2C)
#define LSM6DSR_OUTZ_H_A (0x2D)

#define LSM6DSR_ODR_OFF (0x00)
#define LSM6DSR_ODR_1P6HZ (0xB0)
#define LSM6DSR_ODR_12P5HZ (0x10)
#define LSM6DSR_ODR_26HZ (0x20)
#define LSM6DSR_ODR_52HZ (0x30)
#define LSM6DSR_ODR_104HZ (0x40)
#define LSM6DSR_ODR_208HZ (0x50)
#define LSM6DSR_ODR_416HZ (0x60)
#define LSM6DSR_ODR_833HZ (0x70)
#define LSM6DSR_ODR_1P66KHZ (0x80)
#define LSM6DSR_ODR_3P33KHZ (0x90)
#define LSM6DSR_ODR_6P66KHZ (0xA0)

#define LSM6DSR_ACCEL_FS_2G (0x00)
#define LSM6DSR_ACCEL_FS_4G (0x08)
#define LSM6DSR_ACCEL_FS_8G (0x0C)
#define LSM6DSR_ACCEL_FS_16G (0x04)

#define LSM6DSR_GYRO_FS_125DPS (0x02)
#define LSM6DSR_GYRO_FS_250DPS (0x00)
#define LSM6DSR_GYRO_FS_500DPS (0x04)
#define LSM6DSR_GYRO_FS_1000DPS (0x08)
#define LSM6DSR_GYRO_FS_2000DPS (0x0C)
#define LSM6DSR_GYRO_FS_4000DPS (0x01)

#define LSM6DSR_ACCEL_FS2G_MG_PER_LSB (0.061)
#define LSM6DSR_ACCEL_FS4G_MG_PER_LSB (0.122)
#define LSM6DSR_ACCEL_FS8G_MG_PER_LSB (0.244)
#define LSM6DSR_ACCEL_FS16G_MG_PER_LSB (0.488)

#define LSM6DSR_ACCEL_FS2G_G_PER_LSB (LSM6DSR_ACCEL_FS2G_MG_PER_LSB / 1000.0)
#define LSM6DSR_ACCEL_FS4G_G_PER_LSB (LSM6DSR_ACCEL_FS4G_MG_PER_LSB / 1000.0)
#define LSM6DSR_ACCEL_FS8G_G_PER_LSB (LSM6DSR_ACCEL_FS8G_MG_PER_LSB / 1000.0)
#define LSM6DSR_ACCEL_FS16G_G_PER_LSB (LSM6DSR_ACCEL_FS16G_MG_PER_LSB / 1000.0)

#define LSM6DSR_GYRO_FS125_MDPS_PER_LSB (4.375)
#define LSM6DSR_GYRO_FS250_MDPS_PER_LSB (8.75)
#define LSM6DSR_GYRO_FS500_MDPS_PER_LSB (17.50)
#define LSM6DSR_GYRO_FS1000_MDPS_PER_LSB (35.0)
#define LSM6DSR_GYRO_FS2000_MDPS_PER_LSB (70.0)
#define LSM6DSR_GYRO_FS4000_MDPS_PER_LSB (140.0)

#define LSM6DSR_GYRO_FS125_DPS_PER_LSB (LSM6DSR_GYRO_FS125_MDPS_PER_LSB / 1000.0)
#define LSM6DSR_GYRO_FS250_DPS_PER_LSB (LSM6DSR_GYRO_FS250_MDPS_PER_LSB / 1000.0)
#define LSM6DSR_GYRO_FS500_DPS_PER_LSB (LSM6DSR_GYRO_FS500_MDPS_PER_LSB / 1000.0)
#define LSM6DSR_GYRO_FS1000_DPS_PER_LSB (LSM6DSR_GYRO_FS1000_MDPS_PER_LSB / 1000.0)
#define LSM6DSR_GYRO_FS2000_DPS_PER_LSB (LSM6DSR_GYRO_FS2000_MDPS_PER_LSB / 1000.0)
#define LSM6DSR_GYRO_FS4000_DPS_PER_LSB (LSM6DSR_GYRO_FS4000_MDPS_PER_LSB / 1000.0)

#define LSM6DSR_TEMP_DEG_C_PER_LSB (256.0)
#define LSM6DSR_TEMP_OFFSET_DEG_C (25.0)

#define LSM6DSR_GYRO_NOISE_MDPS_PER_ROOTHZ (5.0)
#define LSM6DSR_GYRO_NOISE_DPS_PER_ROOTHZ (LSM6DSR_GYRO_NOISE_MDPS_PER_ROOTHZ / 1000.0)

class LSM6DSR : public IMU
{
public:
    enum AccelerometerRange
    {
        ACCEL_RANGE_2G = LSM6DSR_ACCEL_FS_2G,
        ACCEL_RANGE_4G = LSM6DSR_ACCEL_FS_4G,
        ACCEL_RANGE_8G = LSM6DSR_ACCEL_FS_8G,
        ACCEL_RANGE_16G = LSM6DSR_ACCEL_FS_16G
    };

    enum GyroscopeRange
    {
        GYRO_RANGE_125DPS = LSM6DSR_GYRO_FS_125DPS,
        GYRO_RANGE_250DPS = LSM6DSR_GYRO_FS_250DPS,
        GYRO_RANGE_500DPS = LSM6DSR_GYRO_FS_500DPS,
        GYRO_RANGE_1000DPS = LSM6DSR_GYRO_FS_1000DPS,
        GYRO_RANGE_2000DPS = LSM6DSR_GYRO_FS_2000DPS,
        GYRO_RANGE_4000DPS = LSM6DSR_GYRO_FS_4000DPS
    };

    enum DataRate
    {
        DATA_RATE_OFF = LSM6DSR_ODR_OFF,
        DATA_RATE_1P6HZ = LSM6DSR_ODR_1P6HZ,
        DATA_RATE_12P5HZ = LSM6DSR_ODR_12P5HZ,
        DATA_RATE_26HZ = LSM6DSR_ODR_26HZ,
        DATA_RATE_52HZ = LSM6DSR_ODR_52HZ,
        DATA_RATE_104HZ = LSM6DSR_ODR_104HZ,
        DATA_RATE_208HZ = LSM6DSR_ODR_208HZ,
        DATA_RATE_416HZ = LSM6DSR_ODR_416HZ,
        DATA_RATE_833HZ = LSM6DSR_ODR_833HZ,
        DATA_RATE_1P66KHZ = LSM6DSR_ODR_1P66KHZ,
        DATA_RATE_3P33KHZ = LSM6DSR_ODR_3P33KHZ,
        DATA_RATE_6P66KHZ = LSM6DSR_ODR_6P66KHZ
    };
    
    struct Data_s
    {
        uint8_t status;
        double temperature;
        struct
        {
            double X;
            double Y;
            double Z;
        } gyroscope;
        struct
        {
            double X;
            double Y;
            double Z;
        } accelerometer;
    };
private:
    i2c_inst_t* i2c;
    int timeout_ms;
    double accelScaleFactor = 0;
    double gyroScaleFactor = 0;

    bool write(uint8_t reg, uint8_t value);
    bool read(uint8_t reg, uint8_t &value);
    bool read(uint8_t reg, uint8_t *buffer, size_t count);
public:
    LSM6DSR(i2c_inst_t* i2c, int timeout_ms = 10);
    bool begin(AccelerometerRange accelRange = ACCEL_RANGE_2G, GyroscopeRange gyroRange = GYRO_RANGE_250DPS, DataRate accelRate = DATA_RATE_OFF, DataRate gyroRate = DATA_RATE_OFF);
    bool reset(int timeout_ms = 5 * LSM6DSR_INIT_DELAY_MS);
    bool setAccelerometerRange(AccelerometerRange range);
    bool setGyroscopeRange(GyroscopeRange range);
    bool setDataRate(DataRate accelRate, DataRate gyroRate);
    bool setInterrupts(bool accelDataReadyEnable, bool gyroDataReadyEnable = false);
    bool readData(struct Data_s &result);
    bool readData(vmath::vect_t<double>& accel, vmath::vect_t<double>& gyro) override;
    bool whoAmI(uint8_t &whoami);
};

#endif // LSM6DSR_H
