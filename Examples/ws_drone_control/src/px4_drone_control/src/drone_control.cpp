#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("drone_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        offboard_setpoint_counter_ = 0;
        flying_time_counter_ = 0;
        armed_ = false;

        auto timer_callback = [this]() -> void
        {
            if (offboard_setpoint_counter_ == 10)
            {
                // Change to Offboard mode after 10 setpoints

                std::cout << "Mudando para modo offboard" << std::endl;
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            if (flying_time_counter_ < 100)
            {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11)
            {
                std::cout << "Contador: " << offboard_setpoint_counter_ << std::endl;
                offboard_setpoint_counter_++;
            }

            // stop the counter after reaching 100
            if (flying_time_counter_ < 100)
            {
                std::cout << "Tempo de voo: " << flying_time_counter_ << std::endl;
                flying_time_counter_++;
            }
            else if (flying_time_counter_ < 105)
            {
                // Disarm the vehicle
                this->land();
                flying_time_counter_++;
            }
        };
        timer_ = this->create_wall_timer(200ms, timer_callback);
    }

    void arm();
    void disarm();
    void land();

private:
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;
    uint64_t flying_time_counter_;

    bool armed_; //!< armed state

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

void DroneControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{

    std::cout << "Publicando comando" << std::endl;

    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    vehicle_command_publisher_->publish(msg);
}

void DroneControl::publish_offboard_control_mode()
{

    std::cout << "Publicando modo de controle" << std::endl;
    OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
}

void DroneControl::publish_trajectory_setpoint()
{
    std::cout << "Publicando setpoint" << std::endl;
    TrajectorySetpoint msg{};
    msg.position = {-5.0, -5.0, -5.0};
    msg.yaw = -3.14;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    trajectory_setpoint_publisher_->publish(msg);
}

void DroneControl::arm()
{
    std::cout << "Armando o drone" << std::endl;
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    armed_ = true;
}

void DroneControl::disarm()
{
    std::cout << "Desarmando o drone" << std::endl;
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    armed_ = false;
}

void DroneControl::land()
{
    std::cout << "Pousando o drone" << std::endl;
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

int main(int argc, char *argv[])
{

    std::cout << "Iniciando controle do drone com ROS2" << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControl>());
    rclcpp::shutdown();
    return 0;
}