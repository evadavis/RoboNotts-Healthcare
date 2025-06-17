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
    HappySystemHardware::~HappySystemHardware()
    {
        // Clean up motor controllers
        if (motor_controller_left_ != nullptr) {
            motor_controller_free(motor_controller_left_);
        }
        if (motor_controller_right_ != nullptr) {
            motor_controller_free(motor_controller_right_);
        }
    }

    hardware_interface::CallbackReturn HappySystemHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Initialise the super-class
        const hardware_interface::CallbackReturn init_value = hardware_interface::SystemInterface::on_init(info);
        if (init_value != hardware_interface::CallbackReturn::SUCCESS)
        {
            return init_value;
        }
        
        // Initialize hardware state vectors
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        
        // Initialise the motor controller
        motor_controller_left_ = motor_controller_new(MOTOR_CONTROLLER_LEFT_ID, MOTOR_CONTROLLER_LEFT_ADDRESS);
        if (motor_controller_left_ == nullptr) {
            RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), 
                "Failed to initialize left motor controller on port %s", MOTOR_CONTROLLER_LEFT_ID);
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        motor_controller_right_ = motor_controller_new(MOTOR_CONTROLLER_RIGHT_ID, MOTOR_CONTROLLER_RIGHT_ADDRESS);
        if (motor_controller_right_ == nullptr) {
            RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), 
                "Failed to initialize right motor controller on port %s", MOTOR_CONTROLLER_RIGHT_ID);
            // Clean up left controller
            motor_controller_free(motor_controller_left_);
            motor_controller_left_ = nullptr;
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Motor controllers initialized successfully");
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
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_commands_[i] = 0;
            }
        }
        
        // Initialize motor controllers
        if (motor_controller_left_ != nullptr) {
            if (motor_controller_enable_modbus(motor_controller_left_) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), "Failed to enable modbus for left motor");
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (motor_controller_set_motor_enabled(motor_controller_left_) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), "Failed to enable left motor");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        
        if (motor_controller_right_ != nullptr) {
            if (motor_controller_enable_modbus(motor_controller_right_) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), "Failed to enable modbus for right motor");
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (motor_controller_set_motor_enabled(motor_controller_right_) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), "Failed to enable right motor");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HappySystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Deactivating...");

        // Disable motor controllers
        if (motor_controller_left_ != nullptr) {
            motor_controller_set_motor_disabled(motor_controller_left_);
        }
        
        if (motor_controller_right_ != nullptr) {
            motor_controller_set_motor_disabled(motor_controller_right_);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("HappySystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type HappySystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Read from left motor (assuming it's joint index 0)
        if (hw_velocities_.size() > 0 && motor_controller_left_ != nullptr) {
            int32_t position;
            float velocity;
            
            if (motor_controller_get_position(motor_controller_left_, &position) == 0) {
                hw_positions_[0] = static_cast<double>(position);
            }
            
            if (motor_controller_get_velocity(motor_controller_left_, &velocity) == 0) {
                hw_velocities_[0] = static_cast<double>(velocity);
            }
        }
        
        // Read from right motor (assuming it's joint index 1)
        if (hw_velocities_.size() > 1 && motor_controller_right_ != nullptr) {
            int32_t position;
            float velocity;
            
            if (motor_controller_get_position(motor_controller_right_, &position) == 0) {
                hw_positions_[1] = static_cast<double>(position);
            }
            
            if (motor_controller_get_velocity(motor_controller_right_, &velocity) == 0) {
                hw_velocities_[1] = static_cast<double>(velocity);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type happy::HappySystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Send velocity commands to left motor (assuming it's joint index 0)
        if (hw_commands_.size() > 0 && motor_controller_left_ != nullptr) {
            float velocity_cmd = static_cast<float>(hw_commands_[0]);
            if (motor_controller_set_velocity(motor_controller_left_, velocity_cmd) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), 
                    "Failed to set velocity for left motor");
            }
        }
        
        // Send velocity commands to right motor (assuming it's joint index 1)
        if (hw_commands_.size() > 1 && motor_controller_right_ != nullptr) {
            float velocity_cmd = static_cast<float>(hw_commands_[1]);
            if (motor_controller_set_velocity(motor_controller_right_, velocity_cmd) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("HappySystemHardware"), 
                    "Failed to set velocity for right motor");
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace happy

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(happy::HappySystemHardware, hardware_interface::SystemInterface)