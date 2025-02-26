#include "motor.h"

#include <stdio.h>

#include <hardware/pwm.h>


// https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=2
// Both pwm_set_freq_duty and initMotor, with modification, come from the article above

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d) {

    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 +  (clock % (f * 4096) != 0);

    if (divider16 / 16 == 0) {
        divider16 = 16;
    }
    uint32_t wrap = clock * 16 / divider16 / f - 1;

    pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);
    return wrap;
}

uint32_t initMotor(int* pins, int freq, int duty) {

    gpio_init(pins[1]);
    gpio_set_dir(pins[1], GPIO_OUT);
    gpio_put(pins[1], 0);

    gpio_init(pins[2]);
    gpio_set_dir(pins[2], GPIO_OUT);
    gpio_put(pins[2], 1);

    gpio_set_function(pins[0], GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pins[0]);
    uint chan = pwm_gpio_to_channel(pins[0]);
    uint32_t wrap = pwm_set_freq_duty(slice_num,chan, freq, duty);
    pwm_set_enabled(slice_num, true);

    return wrap;
}

bool changeDuty(float wrap, float duty, int* pins) { 
    uint slice_num = pwm_gpio_to_slice_num(pins[0]);
    uint chan = pwm_gpio_to_channel(pins[0]);

    if(duty > 0){
        gpio_put(pins[1], 0);
        gpio_put(pins[2], 1);
    } else {
        gpio_put(pins[1], 1);
        gpio_put(pins[2], 0);
        duty = -1 * duty;
    }

    pwm_set_chan_level(slice_num, chan, (int)(wrap * duty / 100));

    return true;
}