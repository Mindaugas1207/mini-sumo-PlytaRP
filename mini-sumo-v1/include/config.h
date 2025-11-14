
#ifndef CONFIG_H
#define CONFIG_H

//MOTOR DRIVER
#define MOTOR_DRIVER_FREQUENCY 20000 //20kHz
#define MOTOR_DRIVER_PWMA 16
#define MOTOR_DRIVER_DIRA 18
#define MOTOR_DRIVER_INVA 0
#define MOTOR_DRIVER_PWMB 17
#define MOTOR_DRIVER_DIRB 19
#define MOTOR_DRIVER_INVB 0

#define MOTOR_DRIVER_ENABLE 25
#define START_PIN 26

//SENSORS
#define SENSORS_ENABLE 20


#define SENSOR_LEFT_90 0
#define SENSOR_LEFT_45 1
#define SENSOR_LEFT_27 2
#define SENSOR_LEFT_18 3
#define SENSOR_LEFT_0 4
#define SENSOR_CENTER_0 5
#define SENSOR_RIGHT_0 6
#define SENSOR_RIGHT_18 7
#define SENSOR_RIGHT_27 8
#define SENSOR_RIGHT_45 9
#define SENSOR_RIGHT_90 10

#define SENSOR_PIN1 0 //45 right
#define SENSOR_PIN2 1 //90 right
#define SENSOR_PIN3 2 //0 right
#define SENSOR_PIN4 3 //18 left
#define SENSOR_PIN5 4 //27 left
#define SENSOR_PIN6 9 //center
#define SENSOR_PIN7 15 //27 right
#define SENSOR_PIN8 12 //18 right
#define SENSOR_PIN9 13 //0 left
#define SENSOR_PIN10 11 //90 left
#define SENSOR_PIN11 10 //45 left

#define DISTANCE_SENSOR_COUNT 11

static const uint distance_sensor_pins[DISTANCE_SENSOR_COUNT] = {
    SENSOR_PIN10, //90L
    SENSOR_PIN11, //45L
    SENSOR_PIN5, //27L
    SENSOR_PIN4, //18L
    SENSOR_PIN9, //0L
    SENSOR_PIN6, //0
    SENSOR_PIN3, //0R
    SENSOR_PIN8, //18R
    SENSOR_PIN7, //27R
    SENSOR_PIN1, //45R
    SENSOR_PIN2 //90R
};

static const double sensor_angles[] = {
    -90.0, //90 left
    -45.0, //45 left
    -27.0, //27 left
    -18.0, //18 left
    -5.0, //0 left
    0.0, //center
    5.0, //0 right
    18.0, //18 right
    27.0, //27 right
    45.0, //45 right
    90.0//90 right
};

#define DISTANCE_READ_TIME (33 * 1000)

#define LINE_SENSOR_FRONT_LEFT 0
#define LINE_SENSOR_FRONT_RIGHT 1
#define LINE_SENSOR_BACK 2

#define SENSOR_PIN_LINE_1 5 //right
#define SENSOR_PIN_LINE_2 14 //left
#define SENSOR_PIN_LINE_3 7 //back

#define LINE_SENSOR_COUNT 3

static const uint line_sensor_pins[LINE_SENSOR_COUNT] = {
    SENSOR_PIN_LINE_2, //L
    SENSOR_PIN_LINE_1, //R
    SENSOR_PIN_LINE_3 //B
};

#define RECEIVER_PIN 8
#define FLAG_PIN 6

//LED
#define LED_PIN 24

//IMU
#define IMU_INT_PIN 21

//I2C
#define I2C_PORT i2c1
#define I2C_SDA 22
#define I2C_SCL 23
#define I2C_SPEED 1000000 //400kHz

#define LSM6DSR_I2C I2C_PORT

//DEBUG
#define DEBUG 1
#define DEBUG_UART uart0
#define DEBUG_UART_BAUD_RATE 115200
#define DEBUG_UART_TX_PIN 28
#define DEBUG_UART_RX_PIN 29
#define DEBUG_VBUS_PIN 27

void error_handler(const char* str, int code = 0);

#endif // CONFIG_H
