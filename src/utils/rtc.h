#include <Wire.h>
#include <RTClib.h>

class RTC {
public:
     RTC(int SDA, int SCL);
     void initRtc();

     void showCurrentDate();

     time_t showCurrentTime();

     
private:
     RTC_DS3231 rtc;
     DateTime nowDate = rtc.now();
};