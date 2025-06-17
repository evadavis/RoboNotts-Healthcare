#ifndef HAPPY_SYSTEM_HPP_
#define HAPPY_SYSTEM_HPP_

#define HAPPY_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define HAPPY_HARDWARE_LOCAL __attribute__((visibility("hidden")))

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include <vector>

#include "motor_interface/motor_interface.h"

namespace happy {
    class HappySystemHardware : public hardware_interface::SystemInterface
    {
    private:
        const char* MOTOR_CONTROLLER_LEFT_ID = "/dev/ttyUSB0";
        const u_int8_t MOTOR_CONTROLLER_LEFT_ADDRESS = 0x1;
        const char* MOTOR_CONTROLLER_RIGHT_ID = "/dev/ttyUSB1";
        const u_int8_t MOTOR_CONTROLLER_RIGHT_ADDRESS = 0x2;
        // Store the controllers
        motor_controller_t* motor_controller_left_;
        motor_controller_t* motor_controller_right_;
        
        // Hardware state storage
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_commands_;

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(HappySystemHardware);
        
        ~HappySystemHardware();

        HAPPY_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

        HAPPY_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        HAPPY_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        HAPPY_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        HAPPY_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        HAPPY_HARDWARE_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        HAPPY_HARDWARE_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };
    
}

#endif // HAPPY_SYSTEM_HPP_