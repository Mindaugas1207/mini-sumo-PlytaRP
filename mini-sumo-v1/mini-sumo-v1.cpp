#include <stdio.h>
#include <stdarg.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include "config.h"
#include "system.h"
#include "one_wire.pio.h"
#include "motor_driver.h"

#include "hardware/clocks.h"

#include "pio_one_wire_serial.h"
#include "one_wire.h"
#include "one_wire_device.h"









enum RangeFlags
{
    RANGE_FLAG_VALID = 0x01, // measurement valid
    RANGE_FLAG_WRAP = 0x02, // range wrapped around
    RANGE_FLAG_PHASE = 0x04, // out of phase
    RANGE_FLAG_SIGNAL_LOW = 0x08, // signal too low
    RANGE_FLAG_SIGMA_LOW = 0x10, // sigma too low
    RANGE_FLAG_SIGMA_HIGH = 0x20, // sigma too high
    RANGE_FLAG_MIN = 0x40, // below minimum range
    RANGE_FLAG_MAX = 0x80 // above maximum range
};

enum DetectionMode
{
    DETECTION_MODE_ERROR = -1,
    DETECTION_MODE_ANY_VALID_RANGE = 0,
    DETECTION_MODE_THRESHOLD = 1,
    DETECTION_MODE_WINDOW = 2
};

class DistanceSensor : public OneWireDevice
{
public:
    DistanceSensor(OneWire &oneWire, uint index) : OneWireDevice(oneWire, index)
    {
    }

    /*! \brief Get the range status flags from the last measurement
     * \return RangeFlags bitmask, or 0 on error
     */
    RangeFlags getRangeFlags(void)
    {
        uint flags;
        if (!readRegister(0x00, &flags))
        {
            return (RangeFlags)0;
        }
        return (RangeFlags)flags;
    }

    /*! \brief Get the last measured distance
     * \return Distance in mm, or -1 on error
     */
    int getDistance_mm(void)
    {
        uint distance;
        if (!readRegister(0x01, &distance))
        {
            return -1;
        }
        return (int)distance;
    }

    /*! \brief Get the last measured distance
     * \return Distance in cm, or -1 on error
     */
    int getDistance_cm(void)
    {
        uint distance;
        if (!readRegister(0x02, &distance))
        {
            return -1;
        }
        return (int)distance;
    }

    /*! \brief Get the number of SPADs used in the last measurement
     * \return SPAD count, or -1 on error
    */
    int getSPADCount(void)
    {
        uint spadCount;
        if (!readRegister(0x03, &spadCount))
        {
            return -1;
        }
        return (int)spadCount;
    }

    /*! \brief Get the signal rate from the last measurement
     * \return Signal rate in MCPS (Mega Counts Per Second), or -1 on error
     */
    int getSignalRate_MCPS(void)
    {
        uint signalRate;
        if (!readRegister(0x04, &signalRate))
        {
            return -1;
        }
        return (int)signalRate;
    }

    /*! \brief Get the ambient rate from the last measurement
     * \return Ambient rate in MCPS (Mega Counts Per Second), or -1 on error
     */
    int getAmbientRate_MCPS(void)
    {
        uint ambientRate;
        if (!readRegister(0x05, &ambientRate))
        {
            return -1;
        }
        return (int)ambientRate;
    }

    /*! \brief Get the signal sigma from the last measurement
     * \return Signal sigma in mm, or -1 on error
     */
    int getSignalSigma_mm(void)
    {
        uint signalSigma;
        if (!readRegister(0x06, &signalSigma))
        {
            return -1;
        }
        return (int)signalSigma;
    }

    /*! \brief Get the range check flags
     * \return RangeFlags bitmask, or 0 on error
    */
    RangeFlags getRangeChecks(void)
    {
        uint flags;
        if (!readRegister(0x0C, &flags))
        {
            return (RangeFlags)0;
        }
        return (RangeFlags)flags;
    }

    /*! \brief Get the maximum range distance
     * \return Maximum distance in mm, or -1 on error
    */
    int getRangeMax_mm(void)
    {
        uint maxDistance;
        if (!readRegister(0x0D, &maxDistance))
        {
            return -1;
        }
        return (int)maxDistance;
    }

    /*! \brief Get the minimum range distance
    * \return Minimum distance in mm, or -1 on error
    */
    int getRangeMin_mm(void)
    {
        uint minDistance;
        if (!readRegister(0x0E, &minDistance))
        {
            return -1;
        }
        return (int)minDistance;
    }

    /*! \brief Get the range timing budget
     * \return Timing budget in ms, or -1 on error
    */
    int getRangeTiming_ms(void)
    {
        uint timing;
        if (!readRegister(0x0F, &timing))
        {
            return -1;
        }
        return (int)timing;
    }

    /*! \brief Get the range offset
     * \return Offset in mm, or -1 on error
    */
    int getRangeOffset(void)
    {
        uint offset;
        if (!readRegister(0x10, &offset))
        {
            return -1;
        }
        return (int)offset;
    }

    /*! \brief Get the crosstalk compensation value
     * \return Crosstalk in counts, or -1 on error
    */
    int getRangeXTALK(void)
    {
        uint xtalk;
        if (!readRegister(0x11, &xtalk))
        {
            return -1;
        }
        return (int)xtalk;
    }

    /*! \brief Get the signal limit used in the last measurement
     * \return Signal limit in MCPS (Mega Counts Per Second), or -1 on error
    */
    int getRangeSignalLimit_MCPS(void)
    {
        uint limit;
        if (!readRegister(0x12, &limit))
        {
            return -1;
        }
        return (int)limit;
    }

    /*! \brief Get the sigma limit used in the last measurement
     * \return Sigma limit in mm, or -1 on error
    */
    int getRangeSigmaLimit_mm(void)
    {
        uint limit;
        if (!readRegister(0x13, &limit))
        {
            return -1;
        }
        return (int)limit;
    }

    /*! \brief Set the range check flags
     * \param flags RangeFlags bitmask
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeChecks(RangeFlags flags)
    {
        return writeRegister(0x0C, flags);
    }

    /*! \brief Set the maximum range distance
     * \param maxDistance Maximum distance in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeMax_mm(uint maxDistance)
    {
        return writeRegister(0x0D, maxDistance);
    }

    /*! \brief Set the minimum range distance
     * \param minDistance Minimum distance in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeMin_mm(uint minDistance)
    {
        return writeRegister(0x0E, minDistance);
    }

    /*! \brief Set the range timing budget
     * \param timing Timing budget in ms (10 to 200)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeTiming_ms(uint timing)
    {
        return writeRegister(0x0F, timing);
    }

    /*! \brief Set the range offset
     * \param offset Offset in mm (-32768 to 32767)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeOffset(uint offset)
    {
        return writeRegister(0x10, offset);
    }

    /*! \brief Set the crosstalk compensation value
     * \param xtalk Crosstalk in counts (0 to 32767)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeXTALK(uint xtalk)
    {
        return writeRegister(0x11, xtalk);
    }

    /*! \brief Set the signal limit used in measurements
     * \param limit Signal limit in MCPS (Mega Counts Per Second)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeSignalLimit_MCPS(uint limit)
    {
        return writeRegister(0x12, limit);
    }

    /*! \brief Set the sigma limit used in measurements
     * \param limit Sigma limit in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeSigmaLimit_mm(uint limit)
    {
        return writeRegister(0x13, limit);
    }

    /*! \brief Set the detection mode
     * \param mode DetectionMode value
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionMode(DetectionMode mode)
    {
        if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_ANY_VALID_RANGE)
        {
            return false;
        }
        return writeRegister(0x08, mode);
    }

    /*! \brief Get the detection mode
     * \return DetectionMode value, or DETECTION_MODE_ERROR on error
    */
    DetectionMode getDetectionMode(void)
    {
        uint mode;
        if (!readRegister(0x08, &mode))
        {
            return DETECTION_MODE_ERROR;
        }
        if (mode > DETECTION_MODE_WINDOW || mode < DETECTION_MODE_ANY_VALID_RANGE)
        {
            return DETECTION_MODE_ERROR;
        }
        return (DetectionMode)mode;
    }

    /*! \brief Set whether the detection output is inverted
     * \param inverted true to invert the output, false for normal operation
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionInverted(bool inverted)
    {
        return writeRegister(0x09, inverted ? 1 : 0);
    }

    /*! \brief Get whether the detection output is inverted
     * \param inverted Reference to variable to receive the result
     * \return true if the command was acknowledged, false on error
    */
    bool getDetectionInverted(bool &inverted)
    {
        uint value;
        if (!readRegister(0x09, &value))
        {
            return false;
        }
        inverted = value != 0;
        return true;
    }

    /*! \brief Get the detection output state
     * \param detected Reference to variable to receive the result
     * \return true if the command was acknowledged, false on error
    */
    bool getDetectionState(bool &detected)
    {
        uint value;
        if (!readRegister(0x07, &value))
        {
            return false;
        }
        detected = value != 0;
        return true;
    }

    /*! \brief Get the lower threshold for detection mode
     * \return Lower threshold in mm, or -1 on error
    */
    int getDetectionLowerThreshold_mm(void)
    {
        uint threshold;
        if (!readRegister(0x0A, &threshold))
        {
            return -1;
        }
        return (int)threshold;
    }

    /*! \brief Set the lower threshold for detection mode
     * \param threshold Lower threshold in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionLowerThreshold_mm(uint threshold)
    {
        return writeRegister(0x0A, threshold);
    }

    /*! \brief Get the upper threshold for detection mode
     * \return Upper threshold in mm, or -1 on error
    */
    int getDetectionUpperThreshold_mm(void)
    {
        uint threshold;
        if (!readRegister(0x0B, &threshold))
        {
            return -1;
        }
        return (int)threshold;
    }

    /*! \brief Set the upper threshold for detection mode
     * \param threshold Upper threshold in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionUpperThreshold_mm(uint threshold)
    {
        return writeRegister(0x0B, threshold);
    }
};

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

#define SENSOR_PIN1 15

void io_init(void);

int main()
{
    io_init();
    gpio_init(SENSOR_PIN1);
    gpio_set_dir(SENSOR_PIN1, GPIO_OUT);
    gpio_put(SENSOR_PIN1, 0); //Enable motor driver
    gpio_init(SENSORS_ENABLE);
    gpio_set_dir(SENSORS_ENABLE, GPIO_OUT);
    gpio_put(SENSORS_ENABLE, 1); //Enable motor driver
    sleep_ms(100);
    //driver.init();
    //driver.setSpeed(1.0f, 1.0f);
    //printf("Motor driver OK\n");

    PioOneWireSerial pioOneWireSerial(pio0, 0, 1, SENSOR_PIN1, 9600);
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
    debug_printf("Sensor found!\n");



    while (true)
    {
        int distance = sensor.getDistance_mm();
        RangeFlags flags = sensor.getRangeFlags();
        debug_printf("Distance: %d mm, Flags: 0x%02X\n", distance, flags);
        sleep_ms(100);
    }
        
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

    while (!stdio_usb_connected());
    
    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

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
