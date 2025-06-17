#include "happy_ros2_control/happy_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace happy
{
    hardware_interface::CallbackReturn HappySystemHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Initialise the super-class
        const hardware_interface::CallbackReturn init_value = hardware_interface::SystemInterface::on_init(info);
        if (init_value != hardware_interface::CallbackReturn::SUCCESS)
        {
            return init_value;
        }
        
        // Initialise the motor controller
        motor_controller_left_ = motor_controller_new(MOTOR_CONTROLLER_LEFT_ID, MOTOR_CONTROLLER_LEFT_ADDRESS);
        motor_controller_right_ = motor_controller_new(MOTOR_CONTROLLER_RIGHT_ID, MOTOR_CONTROLLER_RIGHT_ADDRESS);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HappySystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HappySystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn HappySystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Activating ...please wait...");

        // set some default values
        hw_positions_[0] = 
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_commands_[i] = 0;
            }
        }
        

        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HappySystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Deactivating...");

        // TODO
        
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type HappySystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        for (std::size_t i = 0; i < hw_velocities_.size(); i++)
        {
            // Simulate DiffBot wheels's movement as a first-order system
            // Update the joint status: this is a revolute joint without any limit.
            // Simply integrates
            hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

            RCLCPP_INFO(
                rclcpp::get_logger("HappySystemHardware"),
                "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
                hw_velocities_[i], info_.joints[i].name.c_str());
        }
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ros2_control_demo_example_2 ::HappySystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Writing...");

        for (auto i = 0u; i < hw_commands_.size(); i++)
        {
            // Simulate sending commands to the hardware
            RCLCPP_INFO(
                rclcpp::get_logger("HappySystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
                info_.joints[i].name.c_str());

            hw_velocities_[i] = hw_commands_[i];
        }
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Joints successfully written!");
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

} // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(happy::HappySystemHardware, hardware_interface::SystemInterface)