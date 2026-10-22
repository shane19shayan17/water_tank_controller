#include "../include/PID.hpp"

PID::PID(const float &p, const float &i, const float &d, const float &ff, const float &dt)
    : k_p(p), k_i(i), k_d(d), k_ff(ff), delta_t(dt)
{
}

PID::PID(const float &d_t) : PID(0.0, 0.0, 0.0, 0.0, d_t)
{
}

PID::PID() : PID(0.0, 0.0, 0.0, 0.0, 0.0)
{
}

void PID::change_gain_i(float i)
{
    k_i = i;
}

void PID::set_min_pid(const float &m)
{
    min_ = m;
}

void PID::set_max_pid(const float &m)
{
    max_ = m;
}

void PID::set_gains(const float (&gains)[4])
{
    k_p = gains[0];
    k_i = gains[1];
    k_d = gains[2];
    k_ff = gains[3];
}

void PID::set_dt(const float &dt)
{
    delta_t = dt;
}

float PID::clamp(const float &input)
{
    return input > max_   ? max_
           : input < min_ ? min_
                          : input;
}

void PID::reset_integral()
{
    sum_error = 0.0;
}

float PID::get_pid(const float &error, const float &ref)
{
    potential_integral = sum_error + error * delta_t;
    result = (k_ff * ref) + (k_p * error) + (k_i * potential_integral) + (k_d * (error - prev_error) / delta_t);

    if (result >= min_ && result <= max_)
    {
        sum_error = potential_integral;
    }
    else
    {
        result = clamp(result);
    }
    prev_error = error;
    return result;
}