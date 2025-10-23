#include <stdio.h>
#include <stdarg.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/util/queue.h"

#include "config.h"
#include "system.h"
#include "one_wire.pio.h"
#include "motor_driver.h"

#include "hardware/clocks.h"

#include "pio_one_wire_serial.h"
#include "one_wire.h"
#include "one_wire_device.h"
#include "distance_sensor.h"

#include "vmath.h"

#define LSM6DSR_SPINLOCK 0
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
#define GYRO_TO_DPS(raw) (raw * ((140.0 * M_PI) / (180.0 * 1000.0)))

void LSM6DSR_INT1_Callback(uint gpio, uint32_t events);

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

struct LSM6DSR_Data_s
{
    absolute_time_t timestamp;
    vmath::vect_t<double> accelerometer;
    vmath::vect_t<double> gyroscope;
};

queue_t LSM6DSR_DataQueue;

bool LSM6DSR_init(void)
{
    bool error = false;
    uint8_t stat = 0;
    error |= !LSM6DSR_read(0x16, stat);
    printf("0: %d, %d\n",error, stat);
    error |= !LSM6DSR_write(INT1_CTRL, INT1_DRDY_XL_bit | INT1_DRDY_G_bit);
    printf("1: %d\n",error);
    //uint8_t rCTRL3_C = 0;
    //error |= !LSM6DSR_read(CTRL3_C, rCTRL3_C);
    //error |= !LSM6DSR_write(CTRL3_C, rCTRL3_C | BDU_bit);
    //uint8_t rCTRL4_C = 0;
    //error |= !LSM6DSR_read(CTRL4_C, rCTRL4_C);
    //error |= !LSM6DSR_write(CTRL4_C, rCTRL4_C | DRDY_MASK_bit);
    error |= !LSM6DSR_write(CTRL1_XL, 0x74);
    printf("2: %d\n",error);
    error |= !LSM6DSR_write(CTRL2_G, 0x7C);
    printf("3: %d\n",error);

    queue_init_with_spinlock(&LSM6DSR_DataQueue, sizeof(LSM6DSR_Data_s), 64, LSM6DSR_SPINLOCK);

    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, false);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &LSM6DSR_INT1_Callback);

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

void LSM6DSR_INT1_Callback(uint gpio, uint32_t events)
{
    //debug_printf("INT called\n");

    struct LSM6DSR_Data_s new_data;
    new_data.timestamp = get_absolute_time();
    if (LSM6DSR_read(new_data))
    {
        queue_try_add(&LSM6DSR_DataQueue, &new_data);
    }
}

MotorDriver driver(MOTOR_DRIVER_PWMA, MOTOR_DRIVER_DIRA, MOTOR_DRIVER_INVA, MOTOR_DRIVER_PWMB, MOTOR_DRIVER_DIRB, MOTOR_DRIVER_INVB, MOTOR_DRIVER_FREQUENCY, true, true);
/*
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq)
{
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
*/

#define SENSOR_PIN1 0

void io_init(void);

int main()
{
    io_init();
    if (!LSM6DSR_init())
    {
        debug_printf("INIT FAIL\n");
    }
    //LSM6DSR_INT1_Callback(0,0);
    //printf("%d\n", gpio_get(IMU_INT_PIN));
    debug_printf("START\n");
    absolute_time_t timestamp = 0;
    int meas_count = 0;

    while (true)
    {
        if (get_absolute_time() - timestamp > (2000 * 1000))
        {
            LSM6DSR_INT1_Callback(0,0);
        }

        struct LSM6DSR_Data_s new_data;
        if (queue_try_remove(&LSM6DSR_DataQueue, &new_data))
        {
            meas_count++;
            if (new_data.timestamp - timestamp > (100 * 1000))
            {
                timestamp = new_data.timestamp;
                //debug_printf("READ OK\n");
                debug_printf("count %d, pin %d\n", meas_count, gpio_get(IMU_INT_PIN));
                meas_count = 0;
                debug_printf("ax %.4f, ay %.4f, az %.4f, gx %.4f, gy %.4f, gz %.4f\n", new_data.accelerometer.X, new_data.accelerometer.Y, new_data.accelerometer.Z, new_data.gyroscope.X, new_data.gyroscope.Y, new_data.gyroscope.Z);
            }
        }
        else
        {
            //printf("%d\n", gpio_get(IMU_INT_PIN));
        }
/*
        uint8_t stat = 0;
        if (LSM6DSR_read(STATUS_REG, stat))
        {
            debug_printf("STATUS, %d\n", stat);
        }
*/
    }

    // gpio_init(MOTOR_DRIVER_ENABLE);
    // gpio_set_dir(MOTOR_DRIVER_ENABLE, GPIO_OUT);
    // gpio_put(MOTOR_DRIVER_ENABLE, 1); //Enable motor driver
    // driver.init();
    // driver.setSpeed(0.2f, 0.2f);
    // printf("Motor driver OK\n");
    // gpio_init(SENSOR_PIN1);
    // gpio_set_dir(SENSOR_PIN1, GPIO_OUT);
    // gpio_put(SENSOR_PIN1, 0); //Enable motor driver
    /*gpio_init(SENSORS_ENABLE);
    gpio_set_dir(SENSORS_ENABLE, GPIO_OUT);
    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(1000);*/

    /*
        PioOneWireSerial pioOneWireSerial(pio0, 0, 1, SENSOR_PIN1, 115200);
        pioOneWireSerial.init();
        OneWire oneWire(pioOneWireSerial);
        oneWire.init();
        DistanceSensor sensor(oneWire, 0);
        if (!sensor.begin())
        {
            debug_printf("Sensor not found!\n");
            while (true)
            {
                sleep_ms(1000);
            }
        }
        sleep_ms(1000);*/
    /*if (sensor.setSerialBaudRate(DeviceSerialBaudRate::SERIAL_BAUD_9600))
    {
        debug_printf("Sensor index set to 1\n");
    }
    else
    {
        debug_printf("Failed to set sensor index to 1\n");
    }*/
    /*
        while (true)
        {

            int distance = sensor.getDistance_mm();
            DistanceSensor::RangeFlags flags = sensor.getRangeFlags();
            debug_printf("Distance: %d mm, Flags: 0x%02X\n", distance, flags);

            sleep_ms(100);
        }
        */
    while (true)
    {
        printf("OK\n");
        sleep_ms(1000);
    }
}

void io_init(void)
{
#if DEBUG
    // Initialize the UART for debug output
    uart_init(DEBUG_UART, DEBUG_UART_BAUD_RATE);
    gpio_set_function(DEBUG_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_UART_RX_PIN, GPIO_FUNC_UART);
    stdio_init_all();
#endif

    while (!stdio_usb_connected())
        ;

    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    sleep_ms(1000);
    debug_printf("IO OK\n");
}

void debug_printf(const char *format, ...)
{
#if DEBUG
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}
