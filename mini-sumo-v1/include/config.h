
#ifndef CONFIG_H
#define CONFIG_H

//MOTOR DRIVER
#define MOTOR_DRIVER_FREQUENCY 10000 //20kHz
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

#define SENSOR_PIN_LINE_1 5 //right
#define SENSOR_PIN_LINE_2 14 //left
#define SENSOR_PIN_LINE_3 7 //back

#define RECEIVER_PIN 8

//LED
#define LED_PIN 24

//IMU
#define IMU_INT_PIN 21

//I2C
#define I2C_PORT i2c1
#define I2C_SDA 22
#define I2C_SCL 23
#define I2C_SPEED 1000000 //400kHz

//DEBUG
#define DEBUG 1
#define DEBUG_UART uart0
#define DEBUG_UART_BAUD_RATE 115200
#define DEBUG_UART_TX_PIN 28
#define DEBUG_UART_RX_PIN 29
#define DEBUG_VBUS_PIN 27

void error_handler(const char* str, int code = 0);

#endif // CONFIG_H
