#ifndef PID_HPP__
#define PID_HPP__

class PID
{
private:
    float k_p, k_i, k_d, k_ff, delta_t;
    float sum_error{0.0}, prev_error{0.0};
    float potential_integral{0.0};
    float result;
    float max_, min_;

    float clamp(const float &input);

public:
    PID(const float &p, const float &i, const float &d, const float &ff, const float &dt);
    PID(const float &d_t);
    PID();
    float get_pid(const float &error, const float &ref = 0);
    void reset_integral();

    void change_gain_i(float i);
    void set_max_pid(const float &m);
    void set_min_pid(const float &m);
    void set_gains(const float (&gains)[4]);
    void set_dt(const float &dt);
};

#endif