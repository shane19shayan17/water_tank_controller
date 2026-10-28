#include "tank_listener/control_algorithm.hpp"

#include <algorithm>
#include <cmath>

ControlAlgorithm::ControlAlgorithm(){
    Kp = 340.0;
    Ki = 140.0;
    Kd = 0.0;
}

//void ControlAlgorithm::setGains(double Kp , double Ki){
//    Kp = Kp;
//    Ki = Ki;
//}

float ControlAlgorithm::calculate(float current , const ControlParameters &parameters){
    double setpoint = 3.00;

    double error = setpoint - current;

    double dt = 0.1;

    double outlet_flow = std::max(
        0.0,
        parameters.drain_coefficient * std::sqrt(std::max(0.0, setpoint)) + parameters.outlet_disturbance
    );

    double u_eq = 0.0;

    double D = 0.0;

    if (!first_sample)
    {
        D = -Kd * (current - previous_level) / dt;
    }

    previous_level = current;
    first_sample = false;

    double P = Kp * error;


    double new_integral = integral_error + error * dt;

    double I = Ki * new_integral;


    double u = u_eq + P + I + D;


    if (!((u > 100.0 && error > 0.0) ||
          (u < 0.0 && error < 0.0)))
    {
        integral_error = new_integral;
    }


    double valve_percentage = u_eq + P + Ki * integral_error + D;


    valve_percentage = std::clamp(valve_percentage, 0.0, 100.0);


    return static_cast<float>(valve_percentage);
}