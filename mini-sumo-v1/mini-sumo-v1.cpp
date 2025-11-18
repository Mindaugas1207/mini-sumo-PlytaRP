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

#include "lsm6dsr.h"
#include "imu.h"

#include "nvm.h"
#include "pid.h"
#include "async_imu.h"
#include "config.h"
#include "state_machine.h"

struct DistanceSensorConfig_s
{
    uint min = 1;
    uint max = 400;
    int offset = 0;
    double linearCorrection = 1;
    uint timing_ms = 33;
    OneWireDevice::DeviceSerialBaudRate baud = OneWireDevice::DeviceSerialBaudRate::SERIAL_BAUD_115200;
    OneWireDevice::DeviceIOMode ioMode = OneWireDevice::DeviceIOMode::IOMODE_SERIAL;
    uint serialIndex = 0;
};

static const DistanceSensorConfig_s distance_sensor_configurations[DISTANCE_SENSOR_COUNT]
{
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    },
    {
        1,//uint min;
        600,//uint max;
    }
};

LSM6DSR imu = LSM6DSR(LSM6DSR_I2C);

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

void configure_sensors(void);
bool distance_sensor_configure(size_t i, DistanceSensorConfig_s& config);
void calibrate_line_sensor(size_t i, uint duration_ms);
void do_imu_calibration(void);
void do_line_calibration(void);
void sensor_restart_serial(uint pin);
void sensor_restart_digital(uint pin);
bool init_sensors(void);

void io_init(void);

#define deg_to_radian(deg) ((deg) * M_PI / 180.0)
#define rad_to_degree(rad) ((rad) * 180.0 / M_PI)

#define COMMAND_1 (1)
#define COMMAND_2 (2)
#define COMMAND_3 (3)
#define COMMAND_CALIBRATE_LINE (34)
#define COMMAND_CALIBRATE_IMU (41)
#define COMMAND_EXIT (59)

#define LINE_CALIBRATION_TIME (5000)

#define SENSOR_RANGE_LONG 450
#define SENSOR_RANGE_SHORT 100
#define SENSOR_RANGE_PUSH 5

constexpr double alignment_accuracy_deg = 2;
constexpr int alignment_time = 100 * 1000;
constexpr double max_speed = 1.0;
constexpr double min_speed = 0.06;

constexpr double max_turn_speed = 0.5;

constexpr double turn_bias = 0.0; //+ more forward, - more backward
constexpr double turn_linear_ratio = 0.0; //+ more linear, - more turn

constexpr double ramp_frequency = 10000; //Hz
constexpr double ramp_rate = min_speed; //0-1
constexpr double ramp_min = min_speed; //0-1

constexpr double pid_Gain = 0.45 / 180.0;
constexpr double pid_Ti = 0.25; //s
constexpr double pid_Td = 400; //s
constexpr double pid_feedforward = min_speed;
constexpr double pid_Kp = pid_Gain;
constexpr double pid_Ki = pid_Kp / pid_Ti;
constexpr double pid_Kd = pid_Kp * pid_Td;
constexpr double pid_integral_limit = min_speed;
constexpr double pid_min_error = 0;


WS2812_Pio STATUS_Led(pio2, 3, LED_PIN);
MotorDriver driver(MOTOR_DRIVER_PWMA, MOTOR_DRIVER_DIRA, MOTOR_DRIVER_INVA, MOTOR_DRIVER_PWMB, MOTOR_DRIVER_DIRB, MOTOR_DRIVER_INVB, MOTOR_DRIVER_FREQUENCY, true, true, true, false);
Ramp rampA(1.0 / ramp_frequency, ramp_rate, ramp_min);
Ramp rampB(1.0 / ramp_frequency, ramp_rate, ramp_min);
PID turn_pid(pid_Kp, pid_Ki, pid_Kd, pid_integral_limit, pid_min_error);//PID yawPid = PID(0.65/180.0, 0.008, 0.012, 0.07, 0.0);

PioOneWireSerial pioOneWire0(pio0, 0, 1, SENSOR_PIN1, 115200);
PioOneWireSerial pioOneWire1(pio0, 0, 1, RECEIVER_PIN, 9600);
OneWire wire0(pioOneWire0);
OneWire wire1(pioOneWire1);
DistanceSensor distance_sensor(wire0, 0);
IrReceiver receiver(wire1, 0);
OneWireDevice flgaModule(wire1, 0);


class Robot
{
private:
    double driver_out_a_current = 0;
    double driver_out_b_current = 0;

    double yaw_angle_target_deg = 0;

    double linear_speed_current = 0;
    double linear_speed_target = 0;

    bool track_yaw = false;
    bool move_active = false;
    //bool move_linear = false;
    

    absolute_time_t pid_timestamp = 0;
    absolute_time_t move_timestamp = 0;
    absolute_time_t alignment_timestamp = 0;
    int move_timeout = 0;

    double move_phase2_speed = 0;
    double move_phase2_timeout = 0;
    bool move_2phase = false;

    bool aligned = false;

    bool flags_are_down = false;

    bool onLine = false;
    bool lineLeftLast = false;
    double lineRatio = 0;

    bool detected_long = false;
    bool detected_short = false;
    bool detected_front_short = false;
    bool detected_front_long = false;
    bool detected_push = false;

    double detected_angle_long = 0;
    double detected_angle_short = 0;

    bool turn_at_start = false;
    int tactic = 0;
    
    void move_linear(double speed, int period);
    void move_align(double angle, int period);
public:
    bool start_input = false;
    bool stop_input = false;
    bool stop_software = false;
    uint distance_mm[DISTANCE_SENSOR_COUNT] = {};
    bool line_states[LINE_SENSOR_COUNT] = {};
    vmath::vect_t<double> orientation = { 0 };
    double yaw_angle_deg = 0;

    Robot(){}

    bool isLineFront() const { return line_states[LINE_SENSOR_FRONT_LEFT] || line_states[LINE_SENSOR_FRONT_RIGHT]; }
    bool isLineLeft() const { return line_states[LINE_SENSOR_FRONT_LEFT]; }
    bool isLineRight() const { return line_states[LINE_SENSOR_FRONT_RIGHT]; }
    bool isLineBack() const { return line_states[LINE_SENSOR_BACK]; }
    bool isLineLeftLast() const { return lineLeftLast; }

    void init();
    void update_input(void);
    void update_output(void);

    void move(double angle, double linearSpeed = 0, int period = -1, int align_timeout = -1);
    //void moveArc(double endAngle, double curveRate, double linearSpeed, int period = -1);

    //void rotate(double angularSpeed, int period = -1);
    void forward(double speed, int period = -1);
    void backward(double speed, int period = -1);
    void stop(void);

    void flags_up() { flags_are_down = false; flgaModule.writeRegister(2,0); }
    void flags_down() { flags_are_down = true; flgaModule.writeRegister(2,1); }
    bool isFlagsDown() { return flags_are_down; }

    bool isAligned(void) { return aligned; }
    bool isStopped(void) { return stop_input || stop_software; }
    bool isMoveComplete(void) { return move_active; }
    bool isStart(void) { return start_input; }
    bool isStop(void) { return stop_input; }

    int getTactic() { return tactic; }
    void setTactic(int i) { tactic = i; }

    bool isTurnAtStart(void) { return turn_at_start; }
    void setTurnAtStart(bool value) { turn_at_start = value; }

    bool isDetectedShort() { return detected_short; }
    bool isDetectedLong() { return detected_long; }
    bool isDetectedFrontShort() { return detected_front_short; }
    bool isDetectedFrontLong() { return detected_front_long; }
    bool isDetectedPush() { return detected_push; }
    bool getAngleShort() { return detected_angle_short; }
    bool getAngleLong() { return detected_angle_long; }
};

void Robot::move_linear(double speed, int period)
{
    linear_speed_target = speed;
    yaw_angle_target_deg = yaw_angle_deg;
    track_yaw = true;
    move_timeout = period;
    move_active = true;
    move_timestamp = get_absolute_time();
    stop_software = false;
}

void Robot::move_align(double angle, int period)
{
    linear_speed_target = 0;
    yaw_angle_target_deg = vmath::UnwrapAngleDegrees(yaw_angle_deg + angle);
    track_yaw = true;
    move_timeout = period;
    move_active = true;
    move_timestamp = get_absolute_time();
    stop_software = false;
}

void Robot::forward(double speed, int period)
{
    move_2phase = false;
    move_linear(speed, period);
}

void Robot::backward(double speed, int period)
{
    move_2phase = false;
    move_linear(-speed, period);
}

void Robot::move(double angle, double linearSpeed, int period, int align_timeout)
{
    if (linearSpeed != 0)
    {
        move_2phase = true;
        move_phase2_speed = linearSpeed;
        move_phase2_timeout = period;
    }
    else
    {
        move_2phase = false;
    }
    move_align(angle, align_timeout);
}

void Robot::stop(void)
{
    move_2phase = false;
    stop_software = true;
    move_active = false;
    driver_out_a_current = 0;
    driver_out_b_current = 0;
    linear_speed_current = 0;
    turn_pid.reset();
    rampA.reset();
    rampB.reset();
    driver.stop(); //Imediate stop, to reduce delay
}

void Robot::update_output()
{
    auto time = get_absolute_time();
    double deltaT = ((double)absolute_time_diff_us(pid_timestamp, time)) / (1000 * 1000);
    pid_timestamp = time;

    if (move_active)
    {
        if (move_timeout >= 0 && absolute_time_diff_us(move_timestamp, time) > (move_timeout * 1000))
        {
            stop();
        }
        else if (move_2phase)
        {
            if (aligned)
            {
                move_2phase = false;
                move_linear(move_phase2_speed, move_phase2_timeout);
            }
        }
    }

    if (stop_input || stop_software)
    {
        driver_out_a_current = 0;
        driver_out_b_current = 0;
        linear_speed_current = 0;
        turn_pid.reset();
        rampA.reset();
        rampB.reset();
        driver.stop();
    }
    else
    {
        double dtA, dtB;

        if (track_yaw)
        {
            double p = turn_pid.compute_degrees(yaw_angle_target_deg, yaw_angle_deg, deltaT);
            p = p + (p > 0 ? pid_feedforward : -pid_feedforward);
            p = std::clamp(p, -max_turn_speed, max_turn_speed);
            p = p * (1 + linear_speed_current * turn_linear_ratio);

            if (p >= 0)
            {
                dtA = p * (1 + turn_bias);
                dtB = -p * (1 - turn_bias);
            }
            else
            {
                dtA = p * (1 - turn_bias);
                dtB = -p * (1 + turn_bias);
            }
        }
        else
        {
            turn_pid.reset();
            dtA = 0;
            dtB = 0;
        }

        double a_target, b_target;

        if (std::abs(linear_speed_current) < (min_speed / 10))
        {
            a_target = dtA;//+P
            b_target = dtB;//-P
        }
        else
        {
            a_target = linear_speed_current + dtA;//+P
            b_target = linear_speed_current + dtB;//-P

            if (a_target > max_speed)
            {
                b_target -= a_target - max_speed;
            }
            else if (b_target > max_speed)
            {
                a_target -= b_target - max_speed;
            }
            else if (a_target < -max_speed)
            {
                b_target += -a_target - max_speed;
            }
            else if (b_target < -max_speed)
            {
                a_target += -b_target - max_speed;
            }
        }
        
        a_target = std::clamp(a_target, -max_speed, max_speed);
        b_target = std::clamp(b_target, -max_speed, max_speed);
        driver_out_a_current = rampA.compute(driver_out_a_current, a_target);
        driver_out_b_current = rampB.compute(driver_out_b_current, b_target);
        driver_out_a_current = std::clamp(driver_out_a_current, -max_speed, max_speed);
        driver_out_b_current = std::clamp(driver_out_b_current, -max_speed, max_speed);
        driver.setSpeed(driver_out_a_current, driver_out_b_current);
    }
}

void Robot::init()
{
    turn_pid.reset();
    rampA.reset();
    rampB.reset();
    driver.stop();
    pid_timestamp = get_absolute_time();
}

void Robot::update_input(void)
{
    static absolute_time_t distance_timestamp = 0;

    if (absolute_time_diff_us(distance_timestamp, get_absolute_time()) > DISTANCE_READ_TIME)
    {
        distance_timestamp = get_absolute_time();
        detected_long = false;
        detected_short = false;
        double sumLong = 0;
        int divLong = 0;
        double sumShort = 0;
        int divShort = 0;
        for (size_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
        {
            wire0.changePin(distance_sensor_pins[i]);
            int d = distance_sensor.getDistance_mm();
            if (d > 0) distance_mm[i] = d;
            else distance_mm[i] = 0xFFFF;

            if (d < SENSOR_RANGE_SHORT)
            {
                sumShort += sensor_angles[i] + 90;
                divShort += 1;
                detected_short = true;
            }
            else if (d < SENSOR_RANGE_LONG)
            {
                sumLong += sensor_angles[i] + 90;
                divLong += 1;
                detected_long = true;
            }
        }

        if (detected_short && divShort != 0)
        {
            detected_angle_short = (sumShort / divShort) -90;
        }
        else
        {
            detected_angle_short = 0;
        }

        if (detected_long && divLong != 0)
        {
            detected_angle_long = (sumLong / divLong) -90;
        }
        else
        {
            detected_angle_long = 0;
        }

        detected_front_long = (distance_mm[SENSOR_CENTER_0] < SENSOR_RANGE_LONG) || ((distance_mm[SENSOR_LEFT_0] < SENSOR_RANGE_LONG) && (distance_mm[SENSOR_RIGHT_0] < SENSOR_RANGE_LONG));
        detected_front_short = (distance_mm[SENSOR_CENTER_0] < SENSOR_RANGE_SHORT) || ((distance_mm[SENSOR_LEFT_0] < SENSOR_RANGE_SHORT) && (distance_mm[SENSOR_RIGHT_0] < SENSOR_RANGE_SHORT));
        detected_push = (distance_mm[SENSOR_CENTER_0] < SENSOR_RANGE_PUSH) || ((distance_mm[SENSOR_LEFT_0] < SENSOR_RANGE_PUSH) && (distance_mm[SENSOR_RIGHT_0] < SENSOR_RANGE_PUSH));

        if ((distance_mm[SENSOR_LEFT_0] < SENSOR_RANGE_PUSH) && (distance_mm[SENSOR_RIGHT_0] < SENSOR_RANGE_PUSH))
        {
            detected_angle_short = 0;
        }
    }
    
    for (size_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        line_states[i] = gpio_get(line_sensor_pins[i]);
    }

    if (isLineFront())
    {
        if (!isLineLeft())
        {
            lineLeftLast = false;
        }
        else if(!isLineRight())
        {
            lineLeftLast = true;
        }
    }
    else
    {
        lineRatio = 0;
    }

    start_input = gpio_get(START_PIN);
    stop_input = gpio_get(MOTOR_DRIVER_ENABLE);

    async_imu_tryGetOrientation(orientation);
    yaw_angle_deg = -orientation.Yaw(); //Make it so that left is negative, right is positive

    double yaw_delta = std::abs(vmath::UnwrapAngleDegrees(yaw_angle_target_deg - yaw_angle_deg));

    if (aligned)
    {
        if (yaw_delta > alignment_accuracy_deg)
        {
            aligned = false;
        }
        alignment_timestamp = get_absolute_time();
    }
    else
    {
        if (yaw_delta < alignment_accuracy_deg)
        {
            if (absolute_time_diff_us(alignment_timestamp, get_absolute_time()) > alignment_time)
            {
                alignment_timestamp = get_absolute_time();
                aligned = true;
            }
        }
        else
        {
            alignment_timestamp = get_absolute_time();
        }
    }
}

Robot robot;

extern const SM::State initial_state_idle;
extern const SM::State state_stop_robot;

SM::StateMachine sm(&initial_state_idle);

extern const SM::StateCondition line_detected; //reusable line avoidance routine

constexpr double line_back_speed = 0.4;
constexpr int line_back_period = 20; //ms
constexpr double line_turn_angle = 90;
constexpr int line_turn_period = 20; //ms
extern const SM::State line_avoidance_start;
extern const SM::State line_avoidance_step_2;
extern const SM::State line_avoidance_step_3;


extern const SM::State state_move_forward_step_1;
extern const SM::State state_move_forward_step_2;
constexpr double move_forward_s1_speed = 0.4;
constexpr double move_forward_s1_period = 100;
constexpr double move_forward_s2_speed = 0.2;
constexpr double move_forward_s2_period = 1000; //long period, becouse should always reach line first

extern const SM::State search_for_target_test;

constexpr double push_speed = 0.6;

extern const SM::State search_for_target;
extern const SM::State state_track_target_long;
extern const SM::State state_track_target_short;
extern const SM::State state_push;

constexpr double turn_180_angle = 180.0;
constexpr int turn_180_timeout = 100;

extern const SM::State turn_180_or_detect;


const SM::StateCondition line_detected(
    []{return robot.isLineFront();},
    []{ 
        robot.stop();
        sm.jump(&line_avoidance_start); // we jump to the subroutine and we will return to the calling routines start
    }
);

const SM::State line_avoidance_start(
    {
        []{ return !robot.isLineFront(); },
        []{ sm.branch(&line_avoidance_step_2); } // we are off the line branch to next step
    },
    []{ 
        robot.backward(line_back_speed);
    }
);

const SM::State line_avoidance_step_2(
    {
        {
            []{ return robot.isLineFront(); },
            []{ sm.branch_return(); } // we are still on the line go back to begining
        },
        {
            []{ return robot.isMoveComplete(); },
            []{
                robot.stop();
                sm.next(&line_avoidance_step_3);
            }
        }
    },
    []{
        robot.backward(line_back_speed, line_back_period);
    }
);

const SM::State line_avoidance_step_3(
    {
        {
            []{ return robot.isLineFront(); },
            []{
                robot.stop();
                sm.branch_return(); // we are back on the line return to begining
            }
        },
        {
            []{ return robot.isMoveComplete(); },
            []{
                robot.stop();
                sm.branch_return(2); // we are done return out of the routine
            }
        }
    },
    []{ 
        robot.move(robot.isLineLeftLast() ? -line_turn_angle : line_turn_angle, 0, -1, line_turn_period);
    }
);

const SM::State initial_state_idle(
    {
        {
            []{ return robot.isStart(); },
            []{
                switch (robot.getTactic())
                {
                case 0:
                default:
                    if (robot.isTurnAtStart())
                    {
                        robot.setTurnAtStart(false);
                        sm.branch(&turn_180_or_detect);
                    }
                    sm.branch(&search_for_target_test);
                    break;
                }
                
            }
        }
    },
    []{
        robot.stop();
    }
);

const SM::State state_stop_robot(
    {
        []{ return false; },
        []{}
    },
    []{
        robot.stop();
        STATUS_Led.blink(Color::Red(), Color::Black(), 500, 500);
    }
);

const SM::State search_for_target_test(
    {
        line_detected,
        {
            []{ return true; },
            []{ sm.branch(&state_move_forward_step_1); }
        }
    },
    []{
        robot.stop();
    }
);

/*
    Move forward at some speed for some period, then reduce speed
*/
const SM::State state_move_forward_step_1(
    {
        line_detected,
        {
            []{ return robot.isMoveComplete(); },
            []{ sm.next(&state_move_forward_step_2); }
        }
    },
    []{
        robot.forward(move_forward_s1_speed, move_forward_s1_period);
    }
);

const SM::State state_move_forward_step_2(
    {
        line_detected,
        {
            []{ return robot.isMoveComplete(); },
            []{
                robot.stop(); 
                sm.branch_return();
            }
        }
    },
    []{
        robot.forward(move_forward_s2_speed, move_forward_s2_period);
    }
);

const SM::State search_for_target(
    {
        line_detected,
        {
            []{ return robot.isDetectedPush(); },
            []{ sm.branch(&state_push); }
        },
        {
            []{ return robot.isDetectedShort(); },
            []{ sm.branch(&state_track_target_short); }
        },
        {
            []{ return robot.isDetectedLong(); },
            []{ sm.branch(&state_track_target_long); }
        },
        {
            []{ return true; },
            []{ sm.branch(&state_move_forward_step_1); }
        }
    },
    []{
        robot.stop();
    }
);

const SM::State state_track_target_long(
    {
        line_detected,
        {
            []{ return !robot.isDetectedLong() || robot.isDetectedShort() || robot.isDetectedPush(); },
            []{
                robot.stop();
                sm.branch_return();
            }
        },
        {
            []{ return true; },
            []{
                robot.move(robot.getAngleLong());
            }
        }
    },
    []{
        robot.stop();
    }
);

const SM::State state_track_target_short(
    {
        line_detected,
        {
            []{ return robot.isDetectedLong() || !robot.isDetectedShort() || robot.isDetectedPush(); },
            []{
                robot.stop();
                sm.branch_return();
            }
        },
        {
            []{ return true; },
            []{
                robot.move(robot.getAngleShort());
            }
        }
    },
    []{
        robot.stop();
    }
);

const SM::State state_push(
    {
        line_detected,
        {
            []{ return !robot.isDetectedPush(); },
            []{
                robot.stop();
                sm.branch_return();
            }
        }
    },
    []{
        robot.forward(push_speed);
    }
);

const SM::State turn_180_or_detect(
    {
        line_detected,
        {
            []{ return robot.isMoveComplete() || robot.isDetectedLong() || robot.isDetectedShort() || robot.isDetectedPush(); },
            []{
                robot.stop();
                sm.branch_return();
            }
        }
    },
    []{
        robot.move(turn_180_angle, 0, -1, turn_180_timeout);
    }
);

const SM::State state_track_move_forward_step_1(
    {
        line_detected,
        {
            []{ return robot.isMoveComplete(); },
            []{ sm.next(&state_move_forward_step_2); }
        }
    },
    []{
        robot.forward(move_forward_s1_speed, move_forward_s1_period);
    }
);

const SM::State state_track_move_forward_step_2(
    {
        line_detected,
        {
            []{ return robot.isMoveComplete(); },
            []{
                robot.stop(); 
                sm.branch_return();
            }
        }
    },
    []{
        robot.forward(move_forward_s2_speed, move_forward_s2_period);
    }
);

void handle_command(uint command)
{
    debug_printf("%d\n",command);
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
        robot.setTactic(1);
        break;
    case 2:
        robot.setTactic(2);
        break;
    case 3:
        robot.setTactic(3);
        break;
    case 4:
        robot.setTactic(4);
        break;
    case 5:
        robot.setTactic(5);
        break;
    case 10:
        robot.setTurnAtStart(true);
        break;
    case COMMAND_EXIT:
        break;
    default:
        break;
    }
}

void handle_remote(void)
{
    static absolute_time_t rc_timestamp = 0;
    uint command = 0;

    if (receiver.getCommand(&command))
    {
        if (absolute_time_diff_us(rc_timestamp, get_absolute_time()) > (500 * 1000))
        {
            handle_command(command);
            rc_timestamp = get_absolute_time();
        }
    }
}

void hardware_init(void);


int main()
{
    io_init();
    NVM.init(FLASH_SECTOR_SIZE, &config0, sizeof(config0));
    LoadConfig();
    hardware_init();
    robot.init();

    //configure_sensors();

    //gpio_init(6);
    //gpio_set_dir(6, GPIO_IN);
    //gpio_put(6, 0);
    
    //sensors_configure();

    //sensor_configure(SENSOR_PIN6);
    //pioOneWireSerial1.setBaudrate(115200);

    while (true)
    {
        robot.update_input();
        if (!robot.isStart())
        {
            handle_remote();
        }
        if (robot.isStop())
        {
            sm.branch(&state_stop_robot);
        }
        sm.loop();
        robot.update_output();
        STATUS_Led.update();
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

    gpio_init(DEBUG_VBUS_PIN);
    gpio_set_dir(DEBUG_VBUS_PIN, GPIO_IN);
    if (gpio_get(DEBUG_VBUS_PIN))
    {
        sleep_ms(100);
        while (!stdio_usb_connected())
        {
            //wait for usb
            tight_loop_contents();
        }
        sleep_ms(1000);
    }

    gpio_init(START_PIN);
    gpio_set_dir(START_PIN, GPIO_IN);
    gpio_init(MOTOR_DRIVER_ENABLE);
    gpio_set_dir(MOTOR_DRIVER_ENABLE, GPIO_IN);
    gpio_init(SENSORS_ENABLE);
    gpio_set_dir(SENSORS_ENABLE, GPIO_OUT);
    gpio_put(SENSORS_ENABLE, 0);

    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    i2c_clear_bus(I2C_SCL, I2C_SDA, 5);
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, false);
}

void hardware_init(void)
{
    STATUS_Led.init();
    STATUS_Led.setBlocking(Color::Black());

    if (!imu.begin(LSM6DSR::ACCEL_RANGE_16G, LSM6DSR::GYRO_RANGE_4000DPS, LSM6DSR::DATA_RATE_3P33KHZ, LSM6DSR::DATA_RATE_833HZ))
    {
        error_handler("IMU init failed", 0);
    }

    if (!imu.setInterrupts(true))
    {
        error_handler("IMU int failed", 0);
    }

    double beta = sqrt(3.0/4.0) * LSM6DSR_GYRO_NOISE_DPS_PER_ROOTHZ * (M_PI / 180.0) * sqrt(833); // Compute beta
    if (!async_imu_init((IMU*)&imu, config0.accelBias, config0.gyroBias, beta))
    {
        error_handler("Async IMU int failed", 0);
    }

    if (!async_imu_start())
    {
        error_handler("Async IMU start failed", 0);
    }

    pioOneWire0.init();
    pioOneWire1.init();
    wire0.init();
    wire1.init();
    driver.init();

    wire1.changePin(FLAG_PIN);
    if (!flgaModule.begin())
    {
        error_handler("Failed to init flag module", 0);
    }
    if (!flgaModule.restartDevice())
    {
        error_handler("Failed to restart flag module", 0);
    }
    if (!flgaModule.begin())
    {
        error_handler("Failed to init flag module", 1);
    }

    wire1.changePin(RECEIVER_PIN);
    if (!receiver.begin())
    {
        error_handler("Failed to init receiver", 0);
    }
    if (!receiver.restartDevice())
    {
        error_handler("Failed to restart receiver", 0);
    }
    if (!receiver.begin())
    {
        error_handler("Failed to init receiver", 1);
    }
    
    //configure_sensors();

    if (!init_sensors())
    {
        error_handler("Failed to init sensors", 0);
    }
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
    async_imu_stop(); //stop the other core
    config0.LockCode = NVM_CONFG_LOCK_CODE;
    NVM.program();
    async_imu_start(); //start the other core
}

void LoadConfig(void)
{
    NVM.load();

    if (config0.LockCode != NVM_CONFG_LOCK_CODE) {
        config0 = {};
    }
}

void i2c_clear_bus(uint scl, uint sda, int delay)
{
    //5us LOW/HIGH -> 10us period -> 100kHz freq
    gpio_set_dir(scl, false);
    gpio_set_dir(sda, false);
    gpio_put(scl, false);
    gpio_put(sda, false);
    gpio_set_function(scl, GPIO_FUNC_SIO);
    gpio_set_function(sda, GPIO_FUNC_SIO);

    if (!gpio_get(sda)) {
        int sclPulseCount = 0;
        while (sclPulseCount < 9 && !gpio_get(sda)) {
            sclPulseCount++;
            gpio_set_dir(scl, true);
            sleep_us(delay);
            gpio_set_dir(scl, false);
            sleep_us(delay);
        }

        if (gpio_get(sda)) {
            // Bus recovered : send a STOP
            gpio_set_dir(sda, true);
            sleep_us(delay);
            gpio_set_dir(sda, false);
        }
    }

    gpio_set_dir(scl, false);
    gpio_set_dir(sda, false);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_set_function(sda, GPIO_FUNC_I2C); 
}

void do_line_calibration(void)
{
    int sensor_idx = 0;
    
    debug_printf("Line calibration, waiting for index\n");

    STATUS_Led.setBlocking(Color(0x00,0xFF,0xFF));
    
    while(true)
    {
        uint cmd = receiver.getCommandBlocking();
        if (cmd == COMMAND_EXIT)
        {
            debug_printf("Line calibration, exit\n");
            STATUS_Led.setBlocking(Color::Black());
            return;
        }
        else if (cmd == COMMAND_1)
        {
            sensor_idx = 0;
            break;
        }
        else if (cmd == COMMAND_2)
        {
            sensor_idx = 1;
            break;
        }
        else if (cmd == COMMAND_3)
        {
            sensor_idx = 2;
            break;
        }
    }

    debug_printf("Line sensor %d calibration start\n", sensor_idx);

    STATUS_Led.setBlocking(Color::Blue());
    calibrate_line_sensor(sensor_idx, LINE_CALIBRATION_TIME);
    STATUS_Led.setBlocking(Color::Black());

    debug_printf("Line sensor %d calibration completed\n", sensor_idx);
}

void do_imu_calibration(void)
{
    STATUS_Led.setBlocking(Color(0x00,0xFF,0xFF));

    debug_printf("IMU calibration start.\n Keep the robot still for 5 seconds...\n");

    if (!async_imu_calibrate())
    {
        error_handler("IMU calibration failed!", 0);
    }

    if (!async_imu_getBias(config0.accelBias, config0.gyroBias))
    {
        error_handler("Failed to get IMU Bias!", 0);
    }

    debug_printf("Accel bias: X %.4f, Y %.4f, Z %.4f\n", config0.accelBias.X, config0.accelBias.Y, config0.accelBias.Z);
    debug_printf("Gyro bias: X %.4f, Y %.4f, Z %.4f\n", config0.gyroBias.X, config0.gyroBias.Y, config0.gyroBias.Z);

    SaveConfig();

    debug_printf("IMU calibration completed.\n");

    STATUS_Led.setBlocking(Color::Black());
}

void calibrate_line_sensor(size_t i, uint duration_ms)
{
    uint initialBaud = wire0.getBaudrate();
    wire0.setBaudrate(9600);
    uint pin = line_sensor_pins[i];
    sensor_restart_serial(pin);
    wire0.changePin(pin, true);
    LineSensor sensor(wire0, 0);

    if (!sensor.begin())
    {
        error_handler("Line sensor failed to boot!", i);
    }

    int max = 0;
    int min = 2000;
    absolute_time_t timeStart = get_absolute_time();
    while (absolute_time_diff_us(timeStart, get_absolute_time()) < (duration_ms * 1000))
    {
        int val = sensor.getTotalAverage();
        if (val < 0) continue;

        if (val > max) max = val;
        if (val < min) min = val;
        sleep_ms(5);
    }

    int dif = ((max - min) / 2) * 0.8 + min;
    if (!sensor.setDetectionLowerThreshold(dif))
    {
        error_handler("Sensor failed to set lower threshold!\n", i);
    }
    sleep_ms(5);
    if (!sensor.setDetectionUpperThreshold(dif))
    {
        error_handler("Sensor failed to set upper threshold!\n", i);
    }
    sleep_ms(5);
    if (!sensor.saveConfiguration())
    {
        error_handler("Sensor failed to save config!\n", i);
    }
    debug_printf("Line sensor %d, max %d, min %d, dif %d\n", i, max, min, dif);
    sleep_ms(50);
    sensor_restart_digital(pin);
    wire0.setBaudrate(initialBaud);
}

bool distance_sensor_configure(size_t i, const DistanceSensorConfig_s& config)
{
    uint initialBaud = wire0.getBaudrate();
    wire0.setBaudrate(9600);
    uint pin = distance_sensor_pins[i];
    sensor_restart_serial(pin);
    wire0.changePin(pin, true);
    DistanceSensor sensor(wire0, 0);

    bool ok = false;

    do {
        if (!sensor.begin())
        {
            debug_printf("Distance sensor failed to boot!\n", i);
            break;
        }

        if (sensor.setRangeMin_mm(config.min))
        {
            int check = sensor.getRangeMin_mm();

            if (check == config.min)
            {
                debug_printf("Sensor: %d, set range min to %dmm\n", i, config.min);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set range min to %dmm, response %d\n", i, config.min, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set range min to %dmm\n", i, config.min);
            break;
        }

        if (sensor.setRangeMax_mm(config.max))
        {
            int check = sensor.getRangeMax_mm();

            if (check == config.max)
            {
                debug_printf("Sensor: %d, set range max to %dmm\n", i, config.max);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set range max to %dmm, response %d\n", i, config.max, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set range max to %dmm\n", i, config.max);
            break;
        }

        if (sensor.setRangeOffset((uint)config.offset))
        {
            int check = sensor.getRangeOffset();

            if (check == config.offset)
            {
                debug_printf("Sensor: %d, set range offset to %dmm\n", i, config.offset);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set range offset to %dmm, response %d\n", i, config.offset, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set range offset to %dmm\n", i, config.offset);
            break;
        }

        uint lincor = (uint)(config.linearCorrection * 0x8000);

        if (sensor.setRangeLinearCorrection(lincor))
        {
            int check = sensor.getRangeLinearCorrection();

            if (check == lincor)
            {
                debug_printf("Sensor: %d, set range linear correction to %f\n", i, config.linearCorrection);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set range linear correction to %f, response %d\n", i, config.linearCorrection, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set range linear correction to %f\n", i, config.linearCorrection);
            break;
        }

        if (sensor.setRangeTiming_ms(config.timing_ms))
        {
            int check = sensor.getRangeTiming_ms();

            if (check == config.timing_ms)
            {
                debug_printf("Sensor: %d, set range timing to %dms\n", i, config.timing_ms);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set range timing to %dms, response %d\n", i, config.timing_ms, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set range timing to %dms\n", i, config.timing_ms);
            break;
        }

        if (sensor.setSerialBaudRate(config.baud))
        {
            OneWireDevice::DeviceSerialBaudRate check = sensor.getSerialBaudRate();

            if (check == config.baud)
            {
                debug_printf("Sensor: %d, set baud to %d\n", i, config.baud);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set baud to %d, response %d\n", i, config.baud, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set baud to %d\n", i, config.baud);
            break;
        }

        if (sensor.setIOMode(config.ioMode))
        {
            OneWireDevice::DeviceIOMode check = sensor.getIOMode();

            if (check == config.ioMode)
            {
                debug_printf("Sensor: %d, set ioMode to %d\n", i, config.ioMode);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set ioMode to %d, response %d\n", i, config.ioMode, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set ioMode to %d\n", i, config.ioMode);
            break;
        }

        if (sensor.setIndex(config.serialIndex))
        {
            uint check = 0;
            if (!sensor.readIndex(check))
            {
                debug_printf("Sensor: %d, failed to set ioMode to %d\n", i, config.serialIndex);
                break;
            }
            else if (check == config.serialIndex)
            {
                debug_printf("Sensor: %d, set ioMode to %d\n", i, config.serialIndex);
            }
            else
            {
                debug_printf("Sensor: %d, failed to set ioMode to %d, response %d\n", i, config.serialIndex, check);
                break;
            }
        }
        else
        {
            debug_printf("Sensor: %d, failed to set ioMode to %d\n", i, config.serialIndex);
            break;
        }

        if (!sensor.saveConfiguration())
        {
            debug_printf("Sensor: %d, failed to save configuration\n");
            break;
        }
        ok = true;
    } while(0);

    sleep_ms(50);
    sensor_restart_digital(pin);
    wire0.setBaudrate(initialBaud);
    wire0.changePin(pin, true);
    return ok;
}

void configure_sensors(void)
{
    debug_printf("Configuring distance sensors\n");

    for (size_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
    {
        if (!distance_sensor_configure(i, distance_sensor_configurations[i]))
        {
            error_handler("Failed to configure sensor!", i);
        }
    }

    debug_printf("Configuring completed\n");
}

void sensor_restart_serial(uint pin)
{
    gpio_put(SENSORS_ENABLE, 0);
    gpio_deinit(pin);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    sleep_ms(50);
    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(100);
    gpio_set_dir(pin, GPIO_IN);
}

void sensor_restart_digital(uint pin)
{
    gpio_put(SENSORS_ENABLE, 0);
    gpio_deinit(pin);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    sleep_ms(50);
    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(100);
}

bool init_sensors(void)
{
    for (size_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
    {
        gpio_init(distance_sensor_pins[i]);
        gpio_set_dir(line_sensor_pins[i], GPIO_IN);
    }

    for (size_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        gpio_init(line_sensor_pins[i]);
        gpio_set_dir(line_sensor_pins[i], GPIO_IN);
    }

    gpio_put(SENSORS_ENABLE, 1);
    sleep_ms(100);

    bool error = false;

    for (size_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
    {
        wire0.changePin(distance_sensor_pins[i]);
        if (distance_sensor.begin())
        {
            debug_printf("Sensor %d, OK\n", i);
        }
        else
        {
            debug_printf("Sensor %d, FAIL\n", i);
            error = true;
        }
    }

    return error;
}
