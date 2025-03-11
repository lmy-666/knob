#ifndef POWER_H
#define POWER_H

#define PUSH_BUTTON_PIN 5
#define ON_OFF_PIN 18

#include <Arduino.h>
#include <driver/rtc_io.h>

void power_init(void);
void power_off(void);


#endif 




