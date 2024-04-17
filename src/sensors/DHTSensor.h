#include <Arduino.h>

#include <DHT.h>
#include <Adafruit_Sensor.h>

#include "sensors.h"

class DHTSensor : public Sensors {
     private:
          DHT _dht;
          int _type;

     public:
          DHTSensor(int pin, int type) : Sensors(pin), _dht(pin, type) {}
          void setup () override {
               _dht.begin();
          }

          int listen () override {
               float humidity = _dht.readHumidity();
               
               return humidity;
          }

          int temperature () {
               float temperature = _dht.readTemperature();

               return temperature;
          }
};