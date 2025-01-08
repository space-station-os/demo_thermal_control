#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <functional>
#include <thread>

using namespace std::chrono_literals;

class CoolingSystemNode : public rclcpp::Node {
public:
    CoolingSystemNode() : Node("cooling_system_server"), pump1_active_(false), pump2_active_(false), T_coldplate_(26.0), T_water_(4.0), T_ammonia_(-45.0) {
        // Service Server
        cooling_service_ = this->create_service<std_srvs::srv::SetBool>(
            "activate_cooling",
            std::bind(&CoolingSystemNode::handle_cooling_request, this, std::placeholders::_1, std::placeholders::_2));

        // Temperature Publisher
        temperature_publisher_ = this->create_publisher<std_msgs::msg::Float32>("coldplate_temperature", 10);

        RCLCPP_INFO(this->get_logger(), "Cooling System Node Initialized");
    }

private:
    bool pump1_active_;
    bool pump2_active_;
    double T_coldplate_;
    double T_water_;
    double T_ammonia_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr cooling_service_;

    void handle_cooling_request(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                std_srvs::srv::SetBool::Response::SharedPtr response) {
        if (pump1_active_ || pump2_active_) {
            response->success = false;
            response->message = "Pumps are already active!";
            return;
        }

        if (request->data) {
            RCLCPP_INFO(this->get_logger(), "Activating pumps...");
            pump1_active_ = activate_pump1();
            pump2_active_ = activate_pump2();

            if (!pump1_active_ || !pump2_active_) {
                response->success = false;
                response->message = "Failed to activate one or both pumps.";
                return;
            }

            response->success = true;
            response->message = "Cooling system activated successfully.";

            // Simulate cooling process
            simulate_cooling();
        } else {
            response->success = false;
            response->message = "Cooling activation failed. Please check request.";
        }
    }

    bool activate_pump1() {
        RCLCPP_INFO(this->get_logger(), "Pump 1 activated (Water loop)");
        return true;  // Simulate successful activation
    }

    bool activate_pump2() {
        RCLCPP_INFO(this->get_logger(), "Pump 2 activated (Ammonia loop)");
        return true;  // Simulate successful activation
    }

    void simulate_cooling() {
        while (T_coldplate_ > 20.0) {
            // Heat transfer from cold plate to water
            double Q_coldplate = 0.1 * 4186 * (T_coldplate_ - T_water_);  // Water flow rate = 0.1 kg/s
            T_coldplate_ -= Q_coldplate / (2.0 * 900);                    // Cold plate: mass = 2 kg, c = 900 J/kg°C
            T_water_ += Q_coldplate / (0.1 * 4186);

            // Heat transfer from water to ammonia
            if (T_water_ >= 20.0) {
                double Q_water = 0.05 * 4700 * (T_water_ - T_ammonia_);  // Ammonia flow rate = 0.05 kg/s
                T_water_ -= Q_water / (0.1 * 4186);
                T_ammonia_ += Q_water / (0.05 * 4700);
            }

            // Publish cold plate temperature
            auto msg = std_msgs::msg::Float32();
            msg.data = T_coldplate_;
            temperature_publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Cooling: T_coldplate=%.2f°C, T_water=%.2f°C, T_ammonia=%.2f°C",
                        T_coldplate_, T_water_, T_ammonia_);

            std::this_thread::sleep_for(100ms);
        }

        // Deactivate pumps only once temperature reaches 20°C or lower
        pump1_active_ = false;
        pump2_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Operating temperature reached (T_coldplate=%.2f°C). Pumps deactivated.", T_coldplate_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoolingSystemNode>());
    rclcpp::shutdown();
    return 0;
}
