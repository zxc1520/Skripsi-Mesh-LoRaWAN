#ifndef SENSORS_H
#define SENSORS_H

class Sensors {

protected:
     int _pin;

public:
     Sensors(int pin) : _pin(pin) {};
     virtual void setup() = 0;
     virtual int listen() = 0;
     
};

#endif