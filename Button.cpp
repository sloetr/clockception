#include "Button.h"

Button::Button(byte pin) {
    _pin = pin;
    _inverse = false;

    pinMode(_pin,INPUT_PULLUP);
    if(digitalRead(_pin) == 1) _inverse = true; // Assume that button is not pressed on start up to determine inversion
}

bool Button::pushed() {
    if(digitalRead(_pin) == 1) {
        if(_inverse) return false;
        else return true;
    }
    else {
        if(_inverse) return true;
        else return false;
    }
}
