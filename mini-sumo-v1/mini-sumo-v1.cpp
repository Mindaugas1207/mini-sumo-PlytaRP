#include <stdio.h>
#include <stdarg.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/util/queue.h"
#include "pico/mutex.h"
#include "pico/sem.h"
#include "pico/multicore.h"

#include "config.h"
#include "system.h"
#include "one_wire.pio.h"
#include "motor_driver.h"

#include "hardware/clocks.h"

#include "pio_one_wire_serial.h"
#include "one_wire.h"
#include "one_wire_device.h"
#include "distance_sensor.h"
#include "line_sensor.h"
#include "ir_receiver.h"
#include "vmath.h"

#include "pio_ws2812.h"

#include "imu.h"

#include "nvm.hpp"
#include "pid.h"

struct nvm_config_s
{
    vmath::vect_t<double> accelBias;
    vmath::vect_t<double> gyroBias;
    //-----------------------------------//
    uint64_t LockCode;
};
constexpr auto NVM_CONFG_LOCK_CODE = 0xACE9FBD132E3B907;
nvm_config_s config0;

NVM_s NVM;

void SaveConfig(void);
void LoadConfig(void);



void io_init(void);



void flag_down()
{
    gpio_set_dir(6, GPIO_OUT);
    sleep_ms(150);
    gpio_set_dir(6, GPIO_IN);
}


const uint sensor_pins[] = {
    SENSOR_PIN1,
    SENSOR_PIN2,
    SENSOR_PIN3,
    SENSOR_PIN4,
    SENSOR_PIN5,
    SENSOR_PIN6,
    SENSOR_PIN7,
    SENSOR_PIN8,
    SENSOR_PIN9,
    SENSOR_PIN10,
    SENSOR_PIN11,
    SENSOR_PIN_LINE_1,
    SENSOR_PIN_LINE_2,
    SENSOR_PIN_LINE_3
};

#define deg_to_radian(deg) ((deg) * M_PI / 180.0)
#define rad_to_degree(rad) ((rad) * 180.0 / M_PI)

const double sensor_angles[] = {
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

constexpr size_t sensor_count = sizeof(sensor_pins) / sizeof(sensor_pins[0]);
constexpr size_t ls_index = sensor_count - 3;



#define COMMAND_1 (1)
#define COMMAND_2 (2)
#define COMMAND_3 (3)
#define COMMAND_CALIBRATE_LINE (34)
#define COMMAND_CALIBRATE_IMU (41)
#define COMMAND_EXIT (59)

#define LINE_CALIBRATION_TIME (5000)

enum SUMO_STATES
{
    SUMO_IDLE,
    SUMO_FIGHT,
    SUMO_STOP
};

SUMO_STATES sumo_state = SUMO_IDLE;

WS2812_Pio STATUS_Led(pio2, 0, LED_PIN);

PioOneWireSerial pioOneWireSerial0(pio0, 0, 1, RECEIVER_PIN, 9600);
PioOneWireSerial pioOneWireSerial1(pio0, 2, 3, SENSOR_PIN11, 9600);

OneWire oneWire0(pioOneWireSerial0);

IrReceiver receiver(oneWire0, 0);

MotorDriver driver(MOTOR_DRIVER_PWMA, MOTOR_DRIVER_DIRA, MOTOR_DRIVER_INVA, MOTOR_DRIVER_PWMB, MOTOR_DRIVER_DIRB, MOTOR_DRIVER_INVB, MOTOR_DRIVER_FREQUENCY, true, true, true, false);

class RobotState
{
public:
    bool lineLeft = false;
    bool lineRight = false;
    bool lineBack = false;

    bool s_center = false;
    bool s_left0 = false;
    bool s_right0 = false;
    bool s_left18 = false;
    bool s_right18 = false;
    bool s_left27 = false;
    bool s_right27 = false;
    bool s_left45 = false;
    bool s_right45 = false;
    bool s_left90 = false;
    bool s_right90 = false;

    bool s_states[11];

    double target_avg_angle = 0;
    bool target_detected = false;

    bool on_line = false;
    bool turn_after_line = false;
    bool turn_after_line_left = false;
    bool turn_after_line_left_actual = false;
    bool turning_after_line = false;

    bool angle_reached = false;

    bool push = false;

    int dist_center = 0;

    absolute_time_t line_timeout = 0;
    absolute_time_t sensor_timeout = 0;
    absolute_time_t push_timeout = 0;
    vmath::vect_t<double> orientation;

    RobotState(){};
};

RobotState robot_state;

float max_speed = 0.3;
float min_speed = 0.05;

float max_turn_speed = 0.2;
double ramp_cof = 1.0/100000.0;

Ramp rampA = Ramp(ramp_cof, min_speed, min_speed);
Ramp rampB = Ramp(ramp_cof, min_speed, min_speed);
PID yawPid = PID(1.4/M_PI, 0.4, 0.0001, max_speed, 0.0);

//float max_speed = 0.3;

float speedA = 0;
float speedB = 0;

float forwardSpeed = 0;

double target_yaw;
absolute_time_t yaw_timestamp;

bool hold_yaw = false;
bool reversing = false;

double base_speed = 0.2;
double back_speed = 0.4;

int tactic = 0;
void tactic_default();
void tactic_select(int n)
{
    if (n == 0)
    {
        tactic_default();
    }
    if (n == 1)
    {
        base_speed = 0;
    }
    else if (n == 2)
    {
        base_speed = base_speed * 2;
    }
    else
    {
        tactic_default();
    }

}

void tactic_default()
{
    base_speed = 0.2;
}

OneWire oneWireCntr(pioOneWireSerial1);
DistanceSensor sensorCntr(oneWireCntr, 0);

absolute_time_t c_read_tim = 0;

void collect_sensor_data()
{
    robot_state.lineRight = gpio_get(SENSOR_PIN_LINE_1);
    robot_state.lineLeft = gpio_get(SENSOR_PIN_LINE_2);
    robot_state.lineBack = gpio_get(SENSOR_PIN_LINE_3);

    if (absolute_time_diff_us(c_read_tim, get_absolute_time()) > 10000)
    {
        c_read_tim = get_absolute_time();
        robot_state.dist_center = sensorCntr.getDistance_cm();
        if (robot_state.dist_center < 0) robot_state.dist_center = 255;
    }
    
    
    robot_state.s_left90  = robot_state.s_states[0]  = gpio_get(SENSOR_PIN10);
    robot_state.s_left45  = robot_state.s_states[1]  = gpio_get(SENSOR_PIN11);
    robot_state.s_left27  = robot_state.s_states[2]  = gpio_get(SENSOR_PIN5);
    robot_state.s_left18  = robot_state.s_states[3]  = gpio_get(SENSOR_PIN4);
    robot_state.s_left0   = robot_state.s_states[4]  = gpio_get(SENSOR_PIN9);
    robot_state.s_center  = robot_state.dist_center < 40;//robot_state.s_states[5]  = gpio_get(SENSOR_PIN6);
    robot_state.s_right0  = robot_state.s_states[6]  = gpio_get(SENSOR_PIN3);
    robot_state.s_right18 = robot_state.s_states[7]  = gpio_get(SENSOR_PIN8);
    robot_state.s_right27 = robot_state.s_states[8]  = gpio_get(SENSOR_PIN7);
    robot_state.s_right45 = robot_state.s_states[9]  = gpio_get(SENSOR_PIN1);
    robot_state.s_right90 = robot_state.s_states[10] = gpio_get(SENSOR_PIN2);

    bool target_detected =        robot_state.s_center || robot_state.s_left0 ||
                                  robot_state.s_right0 || robot_state.s_left18 ||
                                  robot_state.s_right18 || robot_state.s_left27 ||
                                  robot_state.s_right27 || robot_state.s_left45 ||
                                  robot_state.s_right45 || robot_state.s_left90 ||
                                  robot_state.s_right90;

    double angle_avg = 0;

    if (!robot_state.on_line)
    {
        if (robot_state.lineLeft && !robot_state.lineRight)
        {
            robot_state.turn_after_line_left = true;
        }
    }

    if (robot_state.lineLeft || robot_state.lineRight)
    {
        robot_state.on_line = true;
        robot_state.turn_after_line = true;
        robot_state.turn_after_line_left_actual = robot_state.turn_after_line_left;
        robot_state.line_timeout = get_absolute_time();
    }
    else if (robot_state.on_line)
    {
        if (target_detected)
        {
            robot_state.on_line = false;
            robot_state.turn_after_line_left = false;
        }
        else if (absolute_time_diff_us(robot_state.line_timeout, get_absolute_time()) > (100 * 1000))
        {
            robot_state.on_line = false;
            robot_state.turn_after_line_left = false;
        }
    }

    if (target_detected)
    {
        robot_state.sensor_timeout = get_absolute_time();
    }
    /*
    if (robot_state.s_center) angle_avg = 0;
    else if (robot_state.s_left0 && robot_state.s_right0) angle_avg = 0;
    else if (robot_state.s_left0) angle_avg = deg_to_radian(-12);
    else if (robot_state.s_right0) angle_avg = deg_to_radian(12);
    else if (robot_state.s_left18) angle_avg = deg_to_radian(-18);
    else if (robot_state.s_right18) angle_avg = deg_to_radian(18);
    else if (robot_state.s_left27) angle_avg = deg_to_radian(-27);
    else if (robot_state.s_right27) angle_avg = deg_to_radian(27);
    else if (robot_state.s_left45) angle_avg = deg_to_radian(-45);
    else if (robot_state.s_right45) angle_avg = deg_to_radian(45);
    else if (robot_state.s_left90) angle_avg = deg_to_radian(-90);
    else if (robot_state.s_right90) angle_avg = deg_to_radian(90);
    */
    

    double sum = 0;
    int div = 0;

    for (int i = 0; i < 11; i++)
    {
        if (robot_state.s_states[i])
        {
            sum += sensor_angles[i] + 90;
            div += 1;
        }
    }

    if (!target_detected)
    {
        //if (absolute_time_diff_us(robot_state.sensor_timeout, get_absolute_time()) > (5 * 1000))
        //{
            robot_state.target_detected = false;
        //}
    }
    else
    {
        robot_state.target_detected = true;
        robot_state.target_avg_angle = deg_to_radian((sum / div) -90);
    }

    if (robot_state.s_center)
    {
        if (robot_state.dist_center < 5)
        {
            robot_state.push  = true;
        }
        else
        {
            robot_state.push = false;
        }
/*
        if(absolute_time_diff_us(robot_state.push, get_absolute_time()) > (1000 * 1000))
        {
            robot_state.push  = true;
        }*/
    }
    else
    {
        //robot_state.push_timeout = get_absolute_time();
        robot_state.push = false;
    }

    

    imu_get(robot_state.orientation, 50);

    robot_state.angle_reached = std::abs(vmath::UnwrapAngle(target_yaw - robot_state.orientation.Z)) < 0.1; //[-pi, +pi]
}

absolute_time_t tmstmp;
void test_imu()
{
    

    //while (true)
    //{
    if (absolute_time_diff_us(tmstmp, get_absolute_time()) > (100 * 1000))
    {
        tmstmp = get_absolute_time();
        debug_printf("x %.4f, y %.4f, z %.4f\n", robot_state.orientation.X, robot_state.orientation.Y, robot_state.orientation.Z);
    }
    //}
}

void start_sensor_in_serial(uint pin)
{
    gpio_put(SENSORS_ENABLE, 0);
    gpio_deinit(pin);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    sleep_ms(500);
    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(500);
    gpio_deinit(pin);
}

void start_sensor_as_digital(uint pin)
{
    gpio_deinit(pin);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
}

void select_sensor(uint pin)
{
    pioOneWireSerial1.changePin(pin);
}

void do_line_calibration(void)
{
    int sensor_idx = 0;

    STATUS_Led.set(Color(0x00,0xFF,0xFF));
    STATUS_Led.update();
    
    while(true)
    {
        uint cmd = receiver.getCommandBlocking();
        if (cmd == COMMAND_EXIT) return;
        if (cmd == COMMAND_1)
        {
            sensor_idx = 0;
            break;
        }
        if (cmd == COMMAND_2)
        {
            sensor_idx = 1;
            break;
        }
        if (cmd == COMMAND_3)
        {
            sensor_idx = 2;
            break;
        }
    }
    STATUS_Led.set(Color::Black());
    STATUS_Led.update();
    uint pin = sensor_pins[ls_index + sensor_idx];
    start_sensor_in_serial(pin);
    pioOneWireSerial1.changePin(pin);
    OneWire oneWire(pioOneWireSerial1);
    oneWire.init();
    LineSensor sensor(oneWire, 0);
    if (!sensor.begin())
    {
        error_handler("LS failed to boot in serial", sensor_idx);
    }

    STATUS_Led.blinkBlocking(Color(0x00,0xFF,0xFF), Color::Black(), 100, 100, 10);

    int max = 0;
    int min = 2000;
    STATUS_Led.blink(Color::Blue(), Color::Green(), 100, 100);
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < (LINE_CALIBRATION_TIME * 1000))
    {
        int val = sensor.getTotalAverage();
        if (val < 0) continue;

        if (val > max) max = val;
        if (val < min) min = val;
        sleep_ms(5);
        STATUS_Led.update();
    }

    int dif = ((max - min) / 2) + min;
    sleep_ms(5);
    sensor.setDetectionLowerThreshold(dif);
    sleep_ms(5);
    sensor.setDetectionUpperThreshold(dif);
    sleep_ms(5);
    sensor.saveConfiguration();
    sleep_ms(5);
    sensor.restartDevice();
    sleep_ms(50);
    STATUS_Led.set(Color::Green());
    STATUS_Led.update();

    debug_printf("Line sensor %d, max %d, min %d, dif %d\n", sensor_idx, max, min, dif);

    sleep_ms(500);
    STATUS_Led.set(Color::Black());
    STATUS_Led.update();

    start_sensor_as_digital(pin);
}

void do_imu_calibration(void)
{
    STATUS_Led.set(Color(0x00,0xFF,0xFF));
    STATUS_Led.update();
    debug_printf("imu calib start\n");
    imu_calibrate();
    debug_printf("imu calib end\n");
    //SaveConfig();
    STATUS_Led.set(Color::Black());
    STATUS_Led.update();
}

absolute_time_t rc_timestamp = 0;


void handle_command(uint command)
{
    printf("%d\n",command);
    STATUS_Led.blink(Color::Blue(), Color::Black(), 10, 100, 1);
    switch (command)
    {
    case COMMAND_CALIBRATE_LINE:
        do_line_calibration();
        break;
    case COMMAND_CALIBRATE_IMU:
        do_imu_calibration();
        break;
    case 1:
        tactic = 1;
        break;
        case 2:
        tactic = 2;
        break;
        case 3:
        tactic = 3;
        break;
        case 4:
        tactic = 4;
        break;
        case 5:
        tactic = 5;
        break;
    case COMMAND_EXIT:
        break;
    default:
        break;
    }
}



void set_robot(double speed, double yaw_offset)
{
    target_yaw = vmath::UnwrapAngle(robot_state.orientation.Z + yaw_offset);
    forwardSpeed = speed;
}

void handle_idle(void)
{
    uint command = 0;

    if (gpio_get(START_PIN) && gpio_get(MOTOR_DRIVER_ENABLE))
    {
        pioOneWireSerial1.setBaudrate(115200);
        select_sensor(SENSOR_PIN6);
        
        tactic_select(tactic);
        yaw_timestamp = get_absolute_time();
        target_yaw = robot_state.orientation.Z;
        rampA.reset();
        rampB.reset();
        //flag_down();
        
        sumo_state = SUMO_FIGHT;
        return;
    }

    if (receiver.getCommand(&command))
    {
        if (absolute_time_diff_us(rc_timestamp, get_absolute_time()) > (500 * 1000))
        {
            rc_timestamp = get_absolute_time();
            handle_command(command);
        }
    }

    collect_sensor_data();

    driver.stop();
}

absolute_time_t printfTs = 0;

void handle_fight(void)
{
    if (!gpio_get(START_PIN) || !gpio_get(MOTOR_DRIVER_ENABLE))
    {
        sumo_state = SUMO_STOP;
        STATUS_Led.blink(Color::Red(), Color::Black(), 500, 500);
        return;
    }

    collect_sensor_data();

    reversing = false;

    //set_robot(0, 0);

    if (robot_state.on_line)
    {
        reversing = true;
        set_robot(-back_speed, 0);
        yawPid.reset();
    }
    else if (!robot_state.target_detected)
    {
        if (robot_state.turn_after_line)
        {
            if (robot_state.turn_after_line_left_actual)
            {
                set_robot(0, deg_to_radian(90));
            }
            else
            {
                set_robot(0, deg_to_radian(-90));
            }
            yawPid.reset();
            robot_state.turn_after_line = false;
            robot_state.turning_after_line = true;
        }
        else if (robot_state.turning_after_line)
        {
            if (robot_state.angle_reached)
            {
                robot_state.turning_after_line = false;
                yawPid.reset();
            }
        }
        else
        {
            robot_state.turn_after_line = false;
            robot_state.turning_after_line = false;
            set_robot(base_speed, 0);
        }
    }
    else
    {
        robot_state.turn_after_line = false;
        robot_state.turning_after_line = false;

        if (std::abs(robot_state.target_avg_angle) < 0.1)
        {
            set_robot(base_speed, robot_state.target_avg_angle);
        }
        else
        {
            set_robot(0, robot_state.target_avg_angle);
        }
        
    }
    

    absolute_time_t time = get_absolute_time();
    if (!reversing)
    {
        double deltaT = ((double)absolute_time_diff_us(yaw_timestamp, time)) / (1000 * 1000);
        double p = yawPid.compute_radians(target_yaw, robot_state.orientation.Z, deltaT);
        double df = (1 - std::abs(forwardSpeed)) * p;

        if (isnan(p))
        {
            //printf("%f", deltaT);
            yawPid.reset();
        }
        //speedA = rampA.compute(speedA, forwardSpeed - df);
        //speedB = rampB.compute(speedB, forwardSpeed + df);

        speedA = forwardSpeed + df;
        speedB = forwardSpeed - df;

        if (robot_state.push)
        {
            tactic_default();
            speedA = 0.5;
            speedB = 0.5;
        }
        else
        {
            if (speedA > max_speed) speedA  = max_speed;
            else if (speedA < -max_speed) speedA  = -max_speed;

            if (speedA < min_speed && speedA > -min_speed) speedA = 0;

            if (speedB > max_speed) speedB  = max_speed;
            else if (speedB < -max_speed) speedB  = -max_speed;

            if (speedB < min_speed && speedB > -min_speed) speedB = 0;
        }

        //printf("%d, %f, %f\n", robot_state.target_detected, robot_state.target_avg_angle, p);
        //if (absolute_time_diff_us(printfTs, get_absolute_time()) > (200 * 1000))
        //{
         //printfTs = get_absolute_time();
         //printf("tg %.2f, d %d, A %.2f, T %.2f, p %0.2f, SA %.2f, SB %.2f\n", robot_state.target_avg_angle, robot_state.dist_center, robot_state.orientation.Z,  target_yaw, p, speedA, speedB);
        //}
        //sleep_ms(100);
    }
    else
    {
        speedA = forwardSpeed;
        speedB = forwardSpeed;
        yawPid.reset();
        rampA.reset();
        rampB.reset();
    }
    
    
    yaw_timestamp = time;

    
    
    

    driver.setSpeed(speedA, speedB);
    
/*
    if (robot_state.lineLeft || robot_state.lineRight)
    {
        driver.setSpeed(-0.5f, -0.5f);
    }
    else if (robot_state.s_center)
    {
        driver.setSpeed(0.5f, 0.5f);
    }
    else if (robot_state.s_left90)
    {
        driver.setSpeed(-0.5f, 0.5f);
    }
    else if (robot_state.s_right90)
    {
        driver.setSpeed(0.5f, -0.5f);
    }
    else
    {
        driver.setSpeed(0.0f,0.0f);
    }
*/
}

void handle_stop(void)
{
    driver.stop();
}

void state_machine(void)
{
    switch (sumo_state)
    {
    case SUMO_IDLE:
        handle_idle();
        break;
    case SUMO_FIGHT:
        handle_fight();
        break;
    case SUMO_STOP:
        handle_stop();
        break;
    default:
        break;
    }
}

void sensors_configure(void);
void sensor_configure(uint pin);



int main()
{
    io_init();
    NVM.init(FLASH_SECTOR_SIZE, &config0, sizeof(config0));
    LoadConfig();
    imu_init(config0.accelBias, config0.gyroBias);
    
    pioOneWireSerial1.init();

    //gpio_init(6);
    //gpio_set_dir(6, GPIO_IN);
    //gpio_put(6, 0);
    
    //sensors_configure();

    sensor_configure(6);

  //  while(true)
    //{

    //}

    //sensors_configure();

    start_sensor_as_digital(SENSOR_PIN1);
    start_sensor_as_digital(SENSOR_PIN2);
    start_sensor_as_digital(SENSOR_PIN3);
    start_sensor_as_digital(SENSOR_PIN4);
    start_sensor_as_digital(SENSOR_PIN5);
    start_sensor_as_digital(SENSOR_PIN6);
    start_sensor_as_digital(SENSOR_PIN7);
    start_sensor_as_digital(SENSOR_PIN8);
    start_sensor_as_digital(SENSOR_PIN9);
    start_sensor_as_digital(SENSOR_PIN10);
    start_sensor_as_digital(SENSOR_PIN11);
    start_sensor_as_digital(SENSOR_PIN_LINE_1);
    start_sensor_as_digital(SENSOR_PIN_LINE_2);
    start_sensor_as_digital(SENSOR_PIN_LINE_3);
    gpio_put(SENSORS_ENABLE, 1);
    STATUS_Led.init();
    STATUS_Led.set(Color::Black());
    STATUS_Led.update();
    
    //start_sensors_in_serial();

    pioOneWireSerial0.init();

    oneWireCntr.init();
    
    oneWire0.init();

    driver.init();
    
    if (!receiver.begin()) error_handler("Receiver init", 0);
    if (!receiver.restartDevice()) error_handler("Receiver init", 0);
    if (!receiver.begin()) error_handler("Receiver init", 0);
    

    

    while (true)
    {
        state_machine();
        STATUS_Led.update();
    }
    
    /*
    while (true)
    {
        STATUS_Led.set(Color::Blue());//;
        sleep_ms(100);
        STATUS_Led.set(Color::Black());//);
        sleep_ms(900);
        printf("OK\n");
    }
    */
    //LSM6DSR_INT1_Callback(0,0);
    //printf("%d\n", gpio_get(IMU_INT_PIN));
    //debug_printf("START\n");
    

    

    // gpio_init(MOTOR_DRIVER_ENABLE);
    // gpio_set_dir(MOTOR_DRIVER_ENABLE, GPIO_OUT);
    // gpio_put(MOTOR_DRIVER_ENABLE, 1); //Enable motor driver
    
    //driver.init();
    //driver.setSpeed(0.2f, 0.2f);
    // printf("Motor driver OK\n");
    
    //test_remote();
    /*
    start_sensors_in_serial();
    pioOneWireSerial.init();
    OneWire oneWire(pioOneWireSerial);
    oneWire.init();
    DistanceSensor sensor(oneWire, 0);
    LineSensor lineSensor(oneWire, 0);

    for (int i = ls_index; i < sensor_count; i++)
    {
        sleep_ms(100);
        select_sensor(sensor_pins[i]);
        sleep_ms(100);
        lineSensor.begin();
        sleep_ms(5);
        lineSensor.setDetectionLowerThreshold(950);
        sleep_ms(5);
        lineSensor.setDetectionUpperThreshold(950);
        sleep_ms(5);
        lineSensor.saveConfiguration();
        sleep_ms(5);
        int min = lineSensor.getMin();
        sleep_ms(5);
        int max = lineSensor.getMax();
        sleep_ms(5);
        int lower = lineSensor.getDetectionLowerThreshold();
        sleep_ms(5);
        int upper = lineSensor.getDetectionUpperThreshold();
        sleep_ms(5);

        printf("Sensor %d:\nmin %d, max %d\nlower %d, upper %d\n\n", i+1, min, max, lower, upper);
        
    }

    select_sensor(sensor_pins[ls_index + 2]);
    sleep_ms(100);
    if (!lineSensor.begin())
    {
        debug_printf("line sensor not found!\n");
        error_handler();
    }

    while (true)
    {

        int total_avg = lineSensor.getTotalAverage();
        int avg0 = lineSensor.getS0Average();
        int avg1 = lineSensor.getS1Average();
        int raw0 = lineSensor.getS0Raw();
        int raw1 = lineSensor.getS1Raw();
        DistanceSensor::RangeFlags flags = sensor.getRangeFlags();
        debug_printf("total: %d, avg0: %d, avg1: %d, raw0: %d, raw1: %d\n\n", total_avg, avg0, avg1, raw0, raw1);

        sleep_ms(200);
    }
*/
/*
    for (int i = 0; i < sensor_count; i++)
    {
        sleep_ms(100);
        select_sensor(sensor_pins[i]);
        sleep_ms(100);
        sensor.begin();
        sleep_ms(5);
        //sensor.setRangeMin_mm(1);
        //sleep_ms(5);
       // sensor.setRangeOffset(0);
        //sleep_ms(5);
        ///sensor.setRangeLinearCorrection(0x8000);
        sensor.setRangeMax_mm(800);
        sleep_ms(50);
        sensor.saveConfiguration();
    }


    for (int i = 0; i < sensor_count; i++)
    {
        sleep_ms(100);
        select_sensor(sensor_pins[i]);
        sleep_ms(100);
        sensor.begin();
        
        sleep_ms(5);
        int min = sensor.getRangeMin_mm();
        sleep_ms(5);
        int max = sensor.getRangeMax_mm();
        sleep_ms(5);
        int offset = sensor.getRangeOffset();
        sleep_ms(5);
        int lincor = sensor.getRangeLinearCorrection();
        sleep_ms(5);
        int timing = sensor.getRangeTiming_ms();
        sleep_ms(5);
        int sigma = sensor.getRangeSigmaLimit_mm();
        sleep_ms(5);
        int signal = sensor.getRangeSignalLimit_MCPS();
        sleep_ms(5);
        int xtalk = sensor.getRangeXTALK();
        sleep_ms(5);

        printf("Sensor %d:\nmin %d, max %d\noffset %d\nlincor %d\ntiming %d\nsigma %d\nsignal %d\nxtalk %d\n\n", i+1, min, max, offset, lincor, timing, sigma, signal, xtalk);
    }
*/
    /*
    select_sensor(sensor_pins[5]);
    sleep_ms(100);
    if (!sensor.begin())
    {
        debug_printf("Sensor not found!\n");
        error_handler();
    }
    */
    /*
    if (!sensor.lock())
    {
        debug_printf("Failed loc\n");
        error_handler();
    }
    else
    {
        debug_printf("lock success\n");
    }
    if (!sensor.restartDevice())
    {
        debug_printf("Failed restart\n");
        error_handler();
    }
    else
    {
        debug_printf("restart success\n");
    }
        
    
    */
    /*if (sensor.setSerialBaudRate(DeviceSerialBaudRate::SERIAL_BAUD_9600))
    {
        debug_printf("Sensor index set to 1\n");
    }
    else
    {
        debug_printf("Failed to set sensor index to 1\n");
    }*/
    
    //while (true)
    //{

        //int distance = sensor.getDistance_mm();
        //DistanceSensor::RangeFlags flags = sensor.getRangeFlags();
        //debug_printf("Distance: %d mm, Flags: 0x%02X\n", distance, flags);

    //    sleep_ms(200);
    //}
        
    //while (true)
    //{
    //    printf("OK\n");
    //    sleep_ms(1000);
    //}
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

    while (!stdio_usb_connected());
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);

    gpio_init(START_PIN);
    gpio_set_dir(START_PIN, GPIO_IN);
    gpio_init(MOTOR_DRIVER_ENABLE);
    gpio_set_dir(MOTOR_DRIVER_ENABLE, GPIO_IN);
    gpio_init(SENSORS_ENABLE);
    gpio_set_dir(SENSORS_ENABLE, GPIO_OUT);
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

void error_handler(const char* str, int code)
{
    STATUS_Led.set(Color::Red());
    debug_printf("Error: %d, message: %s\n", code, str);
    while (true)
    {
        STATUS_Led.update();
        sleep_ms(1);
    }
}

void SaveConfig(void)
{
    config0.accelBias = getAccelBias();
    config0.gyroBias = getGyroBias();
    config0.LockCode = NVM_CONFG_LOCK_CODE;
    NVM.program();
}

void LoadConfig(void)
{
    NVM.load();

    if (config0.LockCode != NVM_CONFG_LOCK_CODE) {
        config0 = {};
    }
}


void sensors_configure(void)
{
    gpio_put(SENSORS_ENABLE, 0);
    sleep_ms(500);

    for(int i = 0; i < sensor_count; i++)
    {
        
        gpio_deinit(sensor_pins[i]);
        gpio_init(sensor_pins[i]);
        gpio_set_dir(sensor_pins[i], GPIO_OUT);
        gpio_put(sensor_pins[i], 0);
    }

    sleep_ms(500);
    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(500);

    for (int i = 0; i < sensor_count; i++)
    {
        gpio_deinit(sensor_pins[i]);
    }
/*
    for (int i = 0; i < sensor_count; i++)
    {
        sleep_ms(100);
        select_sensor(sensor_pins[i]);
        sleep_ms(100);
        sensor.begin();
        sleep_ms(5);
        //sensor.setRangeMin_mm(1);
        //sleep_ms(5);
       // sensor.setRangeOffset(0);
        //sleep_ms(5);
        ///sensor.setRangeLinearCorrection(0x8000);
        sensor.setRangeMax_mm(800);
        sleep_ms(50);
        sensor.saveConfiguration();
    }
*/

    OneWire oneWire(pioOneWireSerial1);
    oneWire.init();
    DistanceSensor sensor(oneWire, 0);

    for (int i = 0; i < sensor_count - 3; i++)
    {
        if (sensor_pins[i] == SENSOR_PIN6) continue;
        sleep_ms(500);
        pioOneWireSerial1.changePin(sensor_pins[i]);
        sleep_ms(500);
        sensor.begin();
        sleep_ms(50);
        sensor.setRangeMin_mm(1);
        sleep_ms(50);
        sensor.setRangeMax_mm(400);
        sleep_ms(50);
        sensor.setRangeOffset(0);
        sleep_ms(50);
        sensor.setRangeLinearCorrection(0x8000);
        sleep_ms(50);
        sensor.setIOMode(OneWireDevice::IOMODE_DO);
        sleep_ms(50);
        sensor.saveConfiguration();
        sleep_ms(50);
        sensor.begin();
        sleep_ms(50);
        int min = sensor.getRangeMin_mm();
        sleep_ms(50);
        int max = sensor.getRangeMax_mm();
        sleep_ms(50);
        int offset = sensor.getRangeOffset();
        sleep_ms(50);
        int cor = sensor.getRangeLinearCorrection();
        sleep_ms(50);
        printf("sensor %d, range min %d mm\n", i, min);
        printf("sensor %d, range max %d mm\n", i, max);
        printf("sensor %d, range offset %d mm\n", i, offset);
        printf("sensor %d, range cor %d mm\n", i, cor);

        sensor.setSerialBaudRate(OneWireDevice::SERIAL_BAUD_115200);
        /*
        if (!sensor.lock())
        {
            debug_printf("Failed loc\n");
        }
        else
        {
            debug_printf("lock success\n");
        }
        */
        /*
        int min = sensor.getRangeMin_mm();
        sleep_ms(5);
        int max = sensor.getRangeMax_mm();
        sleep_ms(5);
        int offset = sensor.getRangeOffset();
        sleep_ms(5);
        int lincor = sensor.getRangeLinearCorrection();
        sleep_ms(5);
        int timing = sensor.getRangeTiming_ms();
        sleep_ms(5);
        int sigma = sensor.getRangeSigmaLimit_mm();
        sleep_ms(5);
        int signal = sensor.getRangeSignalLimit_MCPS();
        sleep_ms(5);
        int xtalk = sensor.getRangeXTALK();
        sleep_ms(5);
        */
        //printf("Sensor %d:\nmin %d, max %d\noffset %d\nlincor %d\ntiming %d\nsigma %d\nsignal %d\nxtalk %d\n\n", i+1, min, max, offset, lincor, timing, sigma, signal, xtalk);
    }
    sleep_ms(500);
    gpio_put(SENSORS_ENABLE, 0);
    sleep_ms(500);
}

void sensor_configure(uint pin)
{
    gpio_put(SENSORS_ENABLE, 0);
    sleep_ms(500);

    gpio_deinit(pin);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);

    sleep_ms(500);
    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(500);

    gpio_deinit(pin);
/*
    for (int i = 0; i < sensor_count; i++)
    {
        sleep_ms(100);
        select_sensor(sensor_pins[i]);
        sleep_ms(100);
        sensor.begin();
        sleep_ms(5);
        //sensor.setRangeMin_mm(1);
        //sleep_ms(5);
       // sensor.setRangeOffset(0);
        //sleep_ms(5);
        ///sensor.setRangeLinearCorrection(0x8000);
        sensor.setRangeMax_mm(800);
        sleep_ms(50);
        sensor.saveConfiguration();
    }
*/

    OneWire oneWire(pioOneWireSerial1);
    oneWire.init();
    DistanceSensor sensor(oneWire, 0);

    sleep_ms(500);
    pioOneWireSerial1.changePin(pin);
    sleep_ms(500);
    sensor.begin();
    sleep_ms(50);
    sensor.setRangeMin_mm(1);
    sleep_ms(50);
    sensor.setRangeMax_mm(600);
    sleep_ms(50);
    sensor.setRangeOffset(0);
    sleep_ms(50);
    sensor.setRangeLinearCorrection(0x8000);
    sleep_ms(50);
    sensor.setIOMode(OneWireDevice::IOMODE_DO);
    sleep_ms(50);
    sensor.saveConfiguration();
    sleep_ms(50);
    sensor.begin();
    sleep_ms(50);
    int min = sensor.getRangeMin_mm();
    sleep_ms(50);
    int max = sensor.getRangeMax_mm();
    sleep_ms(50);
    int offset = sensor.getRangeOffset();
    sleep_ms(50);
    int cor = sensor.getRangeLinearCorrection();
    sleep_ms(50);
    printf("Range min %d mm\n", min);
    printf("Range max %d mm\n", max);
    printf("Range offset %d mm\n", offset);
    printf("Range cor %d mm\n", cor);

    if (!sensor.lock())
    {
        debug_printf("Failed loc\n");
    }
    else
    {
        debug_printf("lock success\n");
    }
    sleep_ms(500);
    gpio_put(SENSORS_ENABLE, 0);
    sleep_ms(500);
}
