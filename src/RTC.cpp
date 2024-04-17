#include "utils/rtc.h"

RTC::RTC(int SDA, int SCL) : rtc() {}

void RTC::initRtc() {
     Wire.begin(5, 4);

     rtc.begin();
     rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));

     rtc.adjust(DateTime(2024, 1, 21, 3, 0, 0));
}

void RTC::showCurrentDate () {
     Serial.printf("Tanggal: %d, Bulan: %d, Tahun: %d", nowDate.day(), nowDate.month(), nowDate.year());
}

time_t RTC::showCurrentTime() {
     return nowDate.hour();
}