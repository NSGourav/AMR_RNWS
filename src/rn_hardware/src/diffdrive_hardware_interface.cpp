//on_int() -> Read parameters                -> Unconfigured State
//on_configure() -> Establish Communication  -> Inactive State
//on_activate() -> Engage Actuators          -> Active State


// in Active State read() , write()


#include "diffdrive_hardware/diffdrive_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
// #include "sensor_msgs/msg/imu.hpp" 
#include "pluginlib/class_list_macros.hpp"

namespace diffdrive_hardware
{

hardware_interface::CallbackReturn DiffDriveHardware::on_init(const hardware_interface:: HardwareInfo &info)
{
    if(hardware_interface::SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_=info.hardware_parameters.at("serial_port");
    baud_rate_ =std::stoi(info.hardware_parameters.at("baud_rate"));
    wheel_radius_ =std::stod(info.hardware_parameters.at("wheel_radius"));
    wheel_separation_ =std::stod(info.hardware_parameters.at("wheel_separation"));
    ticks_per_rev_=std::stoi(info.hardware_parameters.at("ticks_per_revolution"));

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),"Initialized with port : %s, baud : %d", serial_port_.c_str(), baud_rate_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_configure(const rclcpp_lifecycle::State &)
{
    serial_fd_=open(serial_port_.c_str(),O_RDWR | O_NOCTTY | O_SYNC);
    sleep(1);
    if(serial_fd_<0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to open Serial Port");
        return hardware_interface::CallbackReturn::ERROR;

    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(serial_fd_,&tty)!=0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to get terminal attributes");
        return hardware_interface::CallbackReturn::ERROR;
    }

    tcflush(serial_fd_, TCIFLUSH);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to set terminal attributes");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}  

hardware_interface::CallbackReturn DiffDriveHardware::on_activate(const rclcpp_lifecycle:: State &)
{
    if (serial_fd_ >= 0) {
        tcflush(serial_fd_, TCIFLUSH);
    }
    cmd_left_vel_ = 0.0;
    cmd_right_vel_ = 0.0;

    imu_pub_ = rclcpp::Node::make_shared("imu_sensor_node")->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);


    // Call write() to push zero velocities to the robot
    auto result = write(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.0));
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(const rclcpp_lifecycle:: State &)
{
    cmd_left_vel_ = 0.0;
    cmd_right_vel_ = 0.0;

    // Call write() to push zero velocities to the robot
    auto result = write(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.0));
    if (result != hardware_interface::return_type::OK)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to send stop command during deactivation");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>DiffDriveHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &left_wheel_pos_);
    state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_);
    state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &right_wheel_pos_);
    state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_);

    // Add IMU sensor state interfaces
    state_interfaces.emplace_back("imu_sensor", "linear_acceleration.x", &imu_acc_x_);
    state_interfaces.emplace_back("imu_sensor", "linear_acceleration.y", &imu_acc_y_);
    state_interfaces.emplace_back("imu_sensor", "linear_acceleration.z", &imu_acc_z_);

    state_interfaces.emplace_back("imu_sensor", "angular_velocity.x", &imu_gyro_x_);
    state_interfaces.emplace_back("imu_sensor", "angular_velocity.y", &imu_gyro_y_);
    state_interfaces.emplace_back("imu_sensor", "angular_velocity.z", &imu_gyro_z_);

    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>DiffDriveHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;    
    command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_left_vel_);
    command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_right_vel_);
    
    return command_interfaces;
}

hardware_interface::return_type DiffDriveHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    char buf[256];
    
    int n= ::read(serial_fd_, buf, sizeof(buf));
    // RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), "Inside read()");
    // std::cout << "[DEBUG] read() called n=" <<n<< std::endl;

    if(n>0)
    {
        serial_buffer_.append(buf,n);

        size_t pos;
        while ((pos = serial_buffer_.find('\n')) != std::string::npos)
        {
            std::string line = serial_buffer_.substr(0,pos);
            serial_buffer_.erase(0, pos+1);
            
            if(line.rfind("ENC", 0)==0)
            {
                std::istringstream ss(line);
                std::string tag;
                int64_t left_ticks, right_ticks;
                ss>>tag>>left_ticks>>right_ticks;

                // RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Ticks: L=%ld R=%ld", left_ticks, right_ticks);
                // std::cout<<"left_ticks: "<<left_ticks<<"   right_ticks: "<<right_ticks<<std::endl;

                double left_revs = static_cast<double>(left_ticks)/ticks_per_rev_;
                double right_revs = static_cast<double>(right_ticks)/ticks_per_rev_;

                left_wheel_pos_ = left_revs * 2 * M_PI;
                right_wheel_pos_ = right_revs * 2 * M_PI;

                double dt = period.seconds();
                left_wheel_vel_ = (left_wheel_pos_ - prev_left_wheel_pos_) / dt;
                right_wheel_vel_ = (right_wheel_pos_ - prev_right_wheel_pos_) / dt;

                prev_left_wheel_pos_ = left_wheel_pos_;
                prev_right_wheel_pos_ = right_wheel_pos_;
            }

            if (line.rfind("IMU", 0) == 0) {
                std::istringstream ss(line);
                std::string tag;
                double qx, qy, qz, qw, ax, ay, az, gx, gy, gz;
            
                ss >> tag >>qx >> qy >> qz >> qw >> gx >> gy >> gz >> ax >> ay >> az;
            
                imu_quat_x_ = qx;
                imu_quat_y_ = qy;
                imu_quat_z_ = qz;
                imu_quat_w_ = qw;

                imu_acc_x_ = ax;
                imu_acc_y_ = ay;
                imu_acc_z_ = az;
            
                imu_gyro_x_ = gx;
                imu_gyro_y_ = gy;
                imu_gyro_z_ = gz;

                // std::cout<<"ax = "<<ax<<std::endl;
            }
        }
        
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = rclcpp::Clock().now();
        imu_msg.header.frame_id = "imu";

        imu_msg.orientation.x = imu_quat_x_;
        imu_msg.orientation.x = imu_quat_y_;
        imu_msg.orientation.x = imu_quat_z_;
        imu_msg.orientation.x = imu_quat_w_;

        imu_msg.linear_acceleration.x = imu_acc_x_;
        imu_msg.linear_acceleration.y = imu_acc_y_;
        imu_msg.linear_acceleration.z = imu_acc_z_;

        imu_msg.angular_velocity.x = imu_gyro_x_;
        imu_msg.angular_velocity.y = imu_gyro_y_;
        imu_msg.angular_velocity.z = imu_gyro_z_;

        // orientation can be filled later if using a sensor like BNO or MPU with DMP

        imu_pub_->publish(imu_msg);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    char cmd_buffer[100];
    double v_left = cmd_left_vel_ * wheel_radius_;
    double v_right = cmd_right_vel_ * wheel_radius_;

    int len=snprintf(cmd_buffer, sizeof(cmd_buffer), "CMD %.3f %.3f\n", v_left, v_right);
    if(len>0)
    {
        int wrote= ::write(serial_fd_, cmd_buffer, len);
        if(wrote<0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to write to serial");
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Cleaning up hardware interface.");
  if (serial_fd_ != -1) 
  {
    close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Serial port closed.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Entered ERROR state.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


}

PLUGINLIB_EXPORT_CLASS(diffdrive_hardware::DiffDriveHardware, hardware_interface::SystemInterface)