#ifndef DIFFDRIVE_HARDWARE_INTERFACE_HPP
#define DIFFDRIVE_HARDWARE_INTERFACE_HPP

#include <vector>
#include <string>
#include <cmath>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "sensor_msgs/msg/imu.hpp" 
#include <rclcpp/rclcpp.hpp>

namespace diffdrive_hardware
{

class DiffDriveHardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;


    std::string serial_port_;
    int baud_rate_;
    double wheel_radius_;
    double wheel_separation_;
    double ticks_per_rev_;

    int64_t left_ticks_count_ =0;
    int64_t right_ticks_count_ =0;
    double left_wheel_pos_ =0.0;
    double right_wheel_pos_=0.0;
    double left_wheel_vel_=0.0;
    double right_wheel_vel_=0.0;
    double prev_left_wheel_pos_=0.0;
    double prev_right_wheel_pos_=0.0;
    std::string serial_buffer_;

    double cmd_left_vel_ = 0.0;
    double cmd_right_vel_ = 0.0;

    double imu_quat_x_ = 0.0;
    double imu_quat_y_ = 0.0;
    double imu_quat_z_ = 0.0;
    double imu_quat_w_ = 0.0;

    double imu_acc_x_ = 0.0;
    double imu_acc_y_ = 0.0;
    double imu_acc_z_ = 0.0;

    double imu_gyro_x_ = 0.0;
    double imu_gyro_y_ = 0.0;
    double imu_gyro_z_ = 0.0;



    int serial_fd_=-1; // file descriptor for serial port
};

}
#endif