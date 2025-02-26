#ifndef MOTOR_H
#define MOTOR_H

#include <pico/stdlib.h>

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);

uint32_t initMotor(int* pins, int freq, int duty);

bool changeDuty(float wrap, float duty, int* pins);

#endif 