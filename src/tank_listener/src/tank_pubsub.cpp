#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "water_tank_interface/msg/tank_input.hpp"
#include "water_tank_interface/msg/tank_output.hpp"

#include "tank_listener/control_algorithm.hpp"

using std::placeholders::_1;

class TankController : public rclcpp::Node{
    public:
        TankController() : Node("tank_controller"){
        //params
        //this->declare_parameter<double>("Kp" , 40);
        //this->declare_parameter<double>("Ki" , 0.1);
        this->declare_parameter<double>("high_goal", 4.5);
        this->declare_parameter<double>("low_goal", 0.5);
        this->declare_parameter<double>("drain_coefficient", 0.04);
        this->declare_parameter<double>("outlet_disturbance", 0.0);
        //pub
        publisher = this->create_publisher<water_tank_interface::msg::TankInput>(
            "/inlet_valve" , 10
        );
        //sub
        subscription = this->create_subscription<water_tank_interface::msg::TankOutput>(
            "/water_level" , 10 , std::bind(&TankController::topic_callback , this , _1)
        );
        }
    private:
       ControlAlgorithm algorithm;

       void topic_callback(const water_tank_interface::msg::TankOutput & msg){
        float current_level = msg.level;
         ControlParameters parameters{
            this->get_parameter("high_goal").as_double(),
            this->get_parameter("low_goal").as_double(),
            this->get_parameter("drain_coefficient").as_double(),
            this->get_parameter("outlet_disturbance").as_double()
        };
        //double Kp = this->get_parameter("Kp").as_double();
        //double Ki = this->get_parameter("Ki").as_double();
        //algorithm.setGains(Kp , Ki);
        float valve_percentage = algorithm.calculate(current_level , parameters);
        //publish msg
        water_tank_interface::msg::TankInput valve_msg;
        valve_msg.inlet_valve = valve_percentage;
        publisher->publish(valve_msg);
         RCLCPP_INFO(
            this->get_logger(),
            "Level: %.2f | High: %.2f | Low: %.2f | Drain: %.3f | Dist: %.3f | Valve: %.1f%%",
            current_level,
            //Kp,
            //Ki,
            parameters.high_goal,
            parameters.low_goal,
            parameters.drain_coefficient,
            parameters.outlet_disturbance,
            valve_percentage
        );
       }
    rclcpp::Publisher<
        water_tank_interface::msg::TankInput
    >::SharedPtr publisher;

    rclcpp::Subscription<
        water_tank_interface::msg::TankOutput
    >::SharedPtr subscription; 
};

int main(int argc , char *argv[]){
    rclcpp::init(argc , argv);
    rclcpp::spin(std::make_shared<TankController>());
    rclcpp::shutdown();
    return 0;
}