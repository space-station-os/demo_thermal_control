#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "thermal_control/thermal_contol/srv/cooling.hpp"
#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TemperaturePublisher : public rclcpp::Node
{
public:
    TemperaturePublisher()
        : Node("temp_publish_node"),
          coldplate_temp_(declare_parameter<double>("coldplate_temperature", 45.0)),
          threshold_temp_(declare_parameter<double>("coldplate_threshold_temperature", 50.0))
    {
        // Publisher
        temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("/coldplate_temperature", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&TemperaturePublisher::publish_temperature, this));

        // Service Client
        cooling_client_ = this->create_client<thermal_control::srv::Cooling>("activate_cooling");

        RCLCPP_INFO(this->get_logger(), "Temperature Publisher Node Initialized");
    }

private:
    double coldplate_temp_;
    double threshold_temp_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cooling_client_;

    void publish_temperature()
    {
        // Simulate temperature increase
        coldplate_temp_ += 0.5;

        // Publish the temperature
        auto msg = sensor_msgs::msg::Temperature();
        msg.temperature = coldplate_temp_;
        msg.variance = 0.0;
        temperature_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Cold Plate Temperature: %.2f°C", coldplate_temp_);



        // Check if cooling is needed
        
        if (coldplate_temp_ >= threshold_temp_) {
            RCLCPP_WARN(this->get_logger(), "Temperature exceeds 50°C. Activating cooling system...");

            activethermalcontrol();
        }

        void activethermalcontrol()
        {

            //Check for server activation
            if (!cooling_client_->wait_for_service(5s))
            {
                RCLCPP_ERROR(this->get_logger(), "Cooling service not available.");
                return;
            }

            auto request = std::make_shared<thermal_control::srv::Cooling::Request>();
            request->temperature = coldplate_temp_;

            auto result_future = cooling_client_->async_send_request(request);
            try{
                auto result =future.get();
                if (result ->success){
                    double pre_temp = coldplate_temp_;
                    coldplate_temp_ = result -> reduced_temperature;
                    threshold_temp_ = coldplate_temp_;
                    if (threshold_temp_<=20){
                        threshold_temp_ = 50;
                    }
                    RCLCPP_INFO(this->get_logger(),
                                " %s coldplate temperature : %.2f deg C.",
                                result->message.c_str(), coldplate_temp_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Cooling process failed: %s", result->message.c_str());
                }

                }
            }
        }   
};
    
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperaturePublisher>());
    rclcpp::shutdown();
    return 0;
}



