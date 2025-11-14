
#include <stdio.h>
#include <stdarg.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "pio_one_wire_serial.h"
#include "one_wire.h"
#include "one_wire_device.h"
#include "distance_sensor.h"
#include "system.h"





/*

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
        sensor.setRangeTiming_ms(33);
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

        //sensor.setSerialBaudRate(OneWireDevice::SERIAL_BAUD_115200);
        
        if (!sensor.lock())
        {
            debug_printf("Failed loc\n");
        }
        else
        {
            debug_printf("lock success\n");
        }
        
        
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
    sensor.setRangeMax_mm(400);
    sleep_ms(50);
    sensor.setRangeOffset(0);
    sleep_ms(50);
    sensor.setRangeLinearCorrection(0x8000);
    sleep_ms(50);
    sensor.setIOMode(OneWireDevice::IOMODE_SERIAL);
    sleep_ms(50);
    sensor.setRangeTiming_ms(33);
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
    auto io = sensor.getIOMode();
    sleep_ms(50);
    printf("Range min %d mm\n", min);
    printf("Range max %d mm\n", max);
    printf("Range offset %d mm\n", offset);
    printf("Range cor %d mm\n", cor);
    printf("IOMode %d\n", (int)io);

    if (!sensor.lock())
    {
        debug_printf("Failed loc\n");
    }
    else
    {
        debug_printf("lock success\n");
    }

    if (!sensor.setSerialBaudRate(OneWireDevice::SERIAL_BAUD_115200))
    {
        debug_printf("Failed set baud\n");
    }
    else
    {
        debug_printf("Set baud success\n");
    }

    while (true)
    {
        int d = sensor.getDistance_cm();
        printf("Distance: %d cm\n", d);
        sleep_ms(200);
    }
    

    sleep_ms(500);
    gpio_put(SENSORS_ENABLE, 0);
    sleep_ms(500);
}

*/
