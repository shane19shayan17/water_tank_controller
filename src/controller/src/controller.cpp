#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "water_tank_interface/msg/tank_input.hpp"
#include "water_tank_interface/msg/tank_output.hpp"

using TankInput = water_tank_interface::msg::TankInput;
using TankOutput = water_tank_interface::msg::TankOutput;

#include "../include/PID.hpp"
#include <array>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
        : Node("controller")
    {
        publisher_ = this->create_publisher<TankInput>(
            "/inlet_valve",
            10);

        sub_tank_output_ = this->create_subscription<TankOutput>(
            "/water_level",
            10,
            std::bind(&ControllerNode::tank_output_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(dt_ms),
            std::bind(&ControllerNode::timer_callback, this));

        /*
            pid config
        */
        level_controller_pid_.set_max_pid(100);
        level_controller_pid_.set_min_pid(0);

        get_pid_params();
        RCLCPP_INFO(get_logger(), "p -> %f", pid_gains_[0]);

        level_controller_pid_.set_gains(pid_gains_);
        level_controller_pid_.set_dt(dt_ms / 1000.0);

        RCLCPP_INFO(get_logger(), "Controller node Ready !!!");
    }

private:
    float water_level{0};
    float error{0};
    float sum_error{0};
    const float d_t{0.1};
    float prev_error{0};
    float k_p{90.0};
    float k_I{0.00005};
    float k_D{0.0};

    void timer_callback()
    {
        tank_input_.inlet_valve = pid(4.15); // sample output
        publisher_->publish(tank_input_);
    }

    void tank_output_callback(const TankOutput::SharedPtr msg)
    {
        water_level = msg->level;
    }

    float pid(float referense)
    {

        error = referense - water_level;
        sum_error += error * d_t;
        float derivate = error / d_t;

        prev_error = error;

        return (k_p * error) + (k_I * sum_error) + (derivate * k_D);
    }

    /*
        pid config
    */
    void get_pid_params()
    {
        this->declare_parameter<float>("k_p", 0.0);
        this->declare_parameter<float>("k_i", 0.0);
        this->declare_parameter<float>("k_d", 0.0);
        this->declare_parameter<float>("k_ff", 0.0);

        this->get_parameter<float>("k_p", pid_gains_[0]);
        this->get_parameter<float>("k_i", pid_gains_[1]);
        this->get_parameter<float>("k_d", pid_gains_[2]);
        this->get_parameter<float>("k_ff", pid_gains_[3]);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TankInput>::SharedPtr publisher_;
    rclcpp::Subscription<TankOutput>::SharedPtr sub_tank_output_;

    TankInput tank_input_;
    PID level_controller_pid_{};
    float pid_gains_[4];
    int dt_ms = 100;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}