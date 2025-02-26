#include "QuadratureEncoder.hpp"

#include "quadrature_encoder.pio.h"

#include <pico/stdlib.h>
#include <hardware/pio.h>


static PIO pio_instance = pio0;
uint offset = pio_add_program(pio_instance, &quadrature_encoder_program);

QuadratureEncoder::QuadratureEncoder(uint8_t pin, uint tmpSM, float ppr, float gear_ratio) : ppr(ppr), gear_ratio(gear_ratio)
{
    sm = tmpSM;
    pio_sm_claim(pio_instance, sm);
    quadrature_encoder_program_init(pio_instance, sm, pin, 0);
}

void QuadratureEncoder::update(float dt)
{
    _last_count = _count;
    _count = quadrature_encoder_get_count(pio_instance, sm);
    _dt = dt;
}

int32_t QuadratureEncoder::get_count()
{
    return _count;
}


float QuadratureEncoder::get_velocity(bool degrees)
{
    float velocity = ((float)(_count - _last_count) / (ppr * gear_ratio)) / _dt;

    if (degrees)
        return 360.0f * velocity; // deg/s
    else
        return 2 * M_PI * velocity; // rad/s
    
}
