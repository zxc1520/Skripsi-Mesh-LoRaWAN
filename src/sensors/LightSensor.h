#include <Arduino.h>

#include "sensors.h"

// Light Sense Pin
#define LIGHT_DO    4

class LightSensor : public Sensors {
     public:
          void setup() override {
               
          }

          int listen() override {
               int value = analogRead(LIGHT_DO);
               
               return value;
          }
};