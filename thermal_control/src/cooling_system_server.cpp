#include <rclcpp/rclcpp.hpp>
#include "thermal_control/srv/cooling.hpp"
#include <functional>
#include <memory>
#include <cmath>

class CoolingSystemNode : public rclcpp::Node
{
public:
    CoolingSystemNode()
        : Node("cooling_system_server"),
          pump1_active_(false)
          
    {
        // Service Server
        cooling_service_ = this->create_service<thermal_control::srv::Cooling>(
            "activate_cooling",
            std::bind(&CoolingSystemNode::handle_cooling_request, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Cooling System Node Initialized");
    }

private:
    bool pump1_active_;
    

    rclcpp::Service<thermal_control::srv::Cooling>::SharedPtr cooling_service_;

    void handle_cooling_request(const thermal_control::srv::Cooling::Request::SharedPtr request,
                                 thermal_control::srv::Cooling::Response::SharedPtr response)
    {
        if (pump1_active_)
        {
            response->success = false;
            response->message = "Cooling system is already active!";
            return;
        }

        double T_coldplate = request->temperature; // Get temperature from client
        double target_temperature = 30.0;         // cooling target (e.g., 30°C)
        double T_water = 4.0;
        double T_ammonia = -40.0;
        double T_env = -270.0; //Space temperature

        //CONSTANTS
        const double c_p_plate = 900;       // Specific heat capacity of aluminum [J/kg*K]
        const double mass_plate = 1;        // Mass of the plate [kg]
        const double eta1 = 0.95;           // Efficiency of plate-water heat exchanger
        const double eta2 = 0.95;           // Efficiency of water-ammonia heat exchanger
        const double c_p_water = 4184;      // Specific heat capacity of water [J/kg*K]
        const double c_p_ammonia = 4500;    // Specific heat capacity of ammonia [J/kg*K]
        const double mass_water = 1;        // Mass of water [kg]
        const double mass_ammonia = 1;      // Mass of ammonia [kg]
        const double h_plate_water = 200;   // Heat transfer coefficient [W/m^2*K]
        const double A_plate_water = 0.1;   // Contact area [m^2]
        const double h_water_ammonia = 150; // Heat transfer coefficient [W/m^2*K]
        const double A_water_ammonia = 0.1; // Contact area [m^2]
        const double sigma = 5.67e-8;       // Stefan-Boltzmann constant [W/m^2*K^4]
        const double epsilon = 0.9;         // Emissivity of radiator
        const double A_radiator = 0.1;      // Radiator area [m^2]


        RCLCPP_INFO(this->get_logger(), "Received cooling request: Current Temp = %.2f°C, Target Temp = %.2f°C",
                    T_coldplate, target_temperature);

        if (T_coldplate <= target_temperature)
        {
            response->success = false;
            response->message = "Cooling not required. Current temperature is already at or below the target.";
            response->reduced_temperature = T_coldplate;
            response->water_temperature = T_water;
            response->ammonia_temperature = T_ammonia;
            return;
        }

        pump1_active_ = true;

        // Heat lost by plate to water
        double Q_plate_water = eta1 * h_plate_water * A_plate_water * (T_coldplate - T_water);

        // Update plate temperature
        double delta_T_plate = Q_plate_water / (mass_plate * c_p_plate);
        T_coldplate -= delta_T_plate;

        // Heat gained by water
        double delta_T_water = Q_plate_water / (mass_water * c_p_water);
        T_water += delta_T_water;

        // Heat transferred from water to ammonia
        double Q_water_ammonia = eta2 * h_water_ammonia * A_water_ammonia * (T_water - T_ammonia);

        // Update water temperature (net heat balance)
        delta_T_water = Q_water_ammonia / (mass_water * c_p_water);
        T_water -= delta_T_water;
        // double Q_water_net = Q_plate_water - Q_water_ammonia; 
        // T_water += Q_water_net / (mass_water * c_p_water);

        //Heat gained by Ammonnia
        double delta_T_ammonia = Q_water_ammonia / (mass_ammonia * c_p_ammonia);
        T_ammonia += delta_T_ammonia;

        //Heat transferred from Ammonia to envronment
        double Q_ammonia_environment = epsilon * sigma * A_radiator * (pow(T_ammonia + 273.15, 4) - pow(T_env + 273.15, 4)) ;

        // Update ammonia temperature
        delta_T_ammonia = Q_ammonia_environment / (mass_ammonia * c_p_ammonia);
        T_ammonia -= delta_T_ammonia;


        // // T_coldplate = simulate_cooling(T_coldplate, target_temperature);
        // const double cooling_rate = 0.1; // Rate of cooling proportional to temperature difference
        // double temp_difference = T_coldplate - T_water_;
        // T_coldplate -= cooling_rate * temp_difference;
        response->reduced_temperature = T_coldplate;
        response->water_temperature = T_water;
        response->ammonia_temperature = T_ammonia;
        response->success = true;
        response->message = "Cooling completed successfully.";
        pump1_active_ = false;
    }

//     double simulate_cooling(double T_coldplate, double target_temperature)
//     {
//         const double cooling_rate = 0.1; // Rate of cooling proportional to temperature difference

//         while (T_coldplate > target_temperature)
//         {
//             double temp_difference = T_coldplate - T_water_;
//             T_coldplate -= cooling_rate * temp_difference;

//             RCLCPP_INFO(this->get_logger(), "Cooling in progress: T_coldplate=%.2f°C", T_coldplate);
//             rclcpp::sleep_for(std::chrono::milliseconds(500));
//         }

//         RCLCPP_INFO(this->get_logger(), "Target temperature reached: %.2f°C", T_coldplate);
//         return T_coldplate;
//     }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoolingSystemNode>());
    rclcpp::shutdown();
    return 0;
}
