#include <Arduino.h>

class Buzzer {
public:
     Buzzer();
     void beepReceive(int pin) {
          digitalWrite(pin, HIGH);
          delay(1000);
          digitalWrite(pin, LOW);
          delay(1000);
     };
     void beepSend(int pin) {
          digitalWrite(pin, HIGH);
          delay(100);
          digitalWrite(pin, LOW);
          delay(100);
          digitalWrite(pin, HIGH);
          delay(100);
          digitalWrite(pin, LOW);
          delay(100);
     };
};

extern Buzzer buzzer;