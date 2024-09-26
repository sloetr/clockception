#ifndef Button_h
#define Button_h

#include <Arduino.h>

class Button
{
private:
    byte _pin;
    bool _inverse;


public:
    Button(byte pin);

    bool pushed();
};

#endif 