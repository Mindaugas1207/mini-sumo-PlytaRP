
#ifndef SYSTEM_H
#define SYSTEM_H

void debug_printf(const char *format, ...);
void i2c_clear_bus(uint scl, uint sda, int delay = 5);

#endif // SYSTEM_H
