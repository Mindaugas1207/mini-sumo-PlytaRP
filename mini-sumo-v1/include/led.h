
#ifndef LED_H
#define LED_H

#include "pico/stdlib.h"

class Color
{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
public:
    Color() : r(0), g(0), b(0), w(0)
    {
        
    }
    Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) : r(r), g(g), b(b), w(w)
    {
    }
    Color(uint32_t grbw) : r(grbw >> 8), g(grbw >> 16), b(grbw), w(grbw >> 24)
    {
    }

    static Color Red(void) {return Color(255,0,0);}
    static Color Green(void) {return Color(0,255,0);}
    static Color Blue(void) {return Color(0,0,255);}
    static Color White(void) {return Color(255,255,255);}
    static Color Black(void) {return Color(0,0,0);}
    static Color None(void) {return Color(0,0,0);}

    uint32_t toGRBW_u32(void)
    {
        return ((uint32_t)w << 24) | ((uint32_t)g << 16) | ((uint32_t)r << 8) | (b);
    }
};

class Led
{
private:
    
public:
    virtual void set(Color c){}
    virtual void setBlocking(Color c){};
    virtual void update(void){}
    virtual void blink(Color color_on, Color color_off, uint period_on, uint period_off, int count){}
    virtual void blinkBlocking(Color color_on, Color color_off, uint period_on, uint period_off, int count){}
};

#endif // LED_H
