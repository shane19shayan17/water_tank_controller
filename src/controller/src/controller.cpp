#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "water_tank_interface/msg/tank_input.hpp"
#include "water_tank_interface/msg/tank_output.hpp"

using TankInput = water_tank_interface::msg::TankInput;
using TankOutput = water_tank_interface::msg::TankOutput;

#include "../include/PID.hpp"
#include <array>

struct OutputMsg
{
    float water_level;
    float desired_level;
    bool running = false;
};

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
            "/tank_output",
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
    void timer_callback()
    {
        if (!tank_output.running)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 250, "waiting for simulator to run !");
            return;
        }

        tank_input_.inlet_valve = compute_pid(tank_output.desired_level, tank_output.water_level);

        publisher_->publish(tank_input_);
    }

    void tank_output_callback(const TankOutput::SharedPtr msg)
    {
        tank_output.water_level = msg->level;
        tank_output.desired_level = msg->desired_level;
        tank_output.running = msg->simulator_running;
    }

    float compute_pid(float reference, float current)
    {
        return level_controller_pid_.get_pid(reference - current, reference);
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
    int dt_s = dt_ms / 1000.0;
    OutputMsg tank_output;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}