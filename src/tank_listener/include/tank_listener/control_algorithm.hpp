#ifndef CONTROL_ALGORITHM_HPP
#define CONTROL_ALGORITHM_HPP

struct ControlParameters
{
    double high_goal;
    double low_goal;
    double drain_coefficient;
    double outlet_disturbance;
};

class ControlAlgorithm
{
private:
    double integral_error = 0.0;
public:
    ControlAlgorithm();
    //void setGains(double Kp , double Ki);
    double previous_level = 0.0;
    bool first_sample = true;
    double Kp;
    double Ki;
    double Kd;
    float calculate(float current_level , const ControlParameters &parameters);
};

#endif