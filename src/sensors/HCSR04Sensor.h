#include <Arduino.h>

#include "sensors.h"

// Proximity Pin
#define ECHO_PIN    13
#define TRIG_PIN    14

class HCSR04Sensor : public Sensors {
private:
     int _trigPin;
     int _echoPin;
public:
     HCSR04Sensor(int trigPin, int echoPin) : Sensors(trigPin) {}
     void setup() override {
          pinMode(_trigPin, OUTPUT);
          pinMode(_echoPin, INPUT);
     }

     int listen() override {
          digitalWrite(TRIG_PIN, LOW);
          delayMicroseconds(5);
          digitalWrite(TRIG_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG_PIN, LOW);

          pinMode(ECHO_PIN, INPUT);
          long duration = pulseIn(ECHO_PIN, HIGH);

          // Convert the time into a distance
          long cm = (duration / 2) / 29.1;

          return cm;
     }
};
