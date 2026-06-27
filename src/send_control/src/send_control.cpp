#include <chrono>
#include <cstdint>
#include <memory>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/point.hpp>

struct Position {
	float x=0;
	float y=0;
	float z=0;
};

auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

class PositionSender : public rclcpp::Node {
public:
	PositionSender() : Node("position_sender") {
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);

		auto current_target_callback = [this](geometry_msgs::msg::Point msg) {
			this->current_target.x = msg.x;
			this->current_target.y = msg.y;
			this->current_target.z = msg.z;
		};
		current_target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("/navigation/current_target", qos, current_target_callback);

		auto timer_callback = [this]() -> void {
			if (this->offboard_setpoint_counter == 10) {
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}
			
			this->publish_offboard_control_mode();
			this->publish_trajectory_setpoint();

			if (this->offboard_setpoint_counter < 11) {
				this->offboard_setpoint_counter++;
			}
		};
		timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
	}

	void arm();
	void disarm();

private:
	int offboard_setpoint_counter = 0;
	Position current_target;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr current_target_subscriber_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1, float param2);
	rcl_time_point_value_t get_timestamp();
};

void PositionSender::arm() {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 0);
	RCLCPP_INFO(this->get_logger(), "Armming");
}

void PositionSender::disarm() {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 0);
	RCLCPP_INFO(this->get_logger(), "Disarmming");
}

void PositionSender::publish_trajectory_setpoint() {
	px4_msgs::msg::TrajectorySetpoint msg;
	msg.position = {current_target.x, current_target.y, current_target.z};
	msg.yaw = -3.14;
	msg.timestamp = this->get_timestamp();
	trajectory_setpoint_publisher_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "CONTROLER: TargetPosition(X:%f, Y:%f, Z:%f)", current_target.x, current_target.y, current_target.z);
}

void PositionSender::publish_offboard_control_mode() {
	px4_msgs::msg::OffboardControlMode msg;
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_timestamp();
	offboard_control_mode_publisher_->publish(msg);
}

void PositionSender::publish_vehicle_command(uint16_t command, float param1, float param2) {
	px4_msgs::msg::VehicleCommand msg;
	msg.command = command;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_timestamp();
	vehicle_command_publisher_->publish(msg);
}

rcl_time_point_value_t PositionSender::get_timestamp() {
	return this->get_clock()->now().nanoseconds() / 1000;
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PositionSender>());
	rclcpp::shutdown();
	return 0;
}
