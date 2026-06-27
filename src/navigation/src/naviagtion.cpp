#include <chrono>
#include <cmath>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#define DEFAULT_EQUIVALENCE_PADDING .1

struct Position {
	float x=0;
	float y=0;
	float z=0;
};

float get_position_difference(const Position &pos1, const Position &pos2) {
	float x_delta = pos1.x - pos2.x;
	float y_delta =pos1.y - pos2.y;
	float z_delta = pos1.z - pos2.z;
	return std::hypot(x_delta, y_delta, z_delta);
}

auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

class Navigation : public rclcpp::Node {
public:
	Navigation() : Node("navigation") {
		this->declare_parameter<float>("equivalence_padding", DEFAULT_EQUIVALENCE_PADDING);
		current_target_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/navigation/current_target", qos);

		auto odom_callback = [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg){
			this->current_position.x = msg->position[0];
			this->current_position.y = msg->position[1];
			this->current_position.z = msg->position[2];
		};
		_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, odom_callback);

		auto timer_callback = [this]() -> void {
			update_waypoints();
			send_current_target();
		};
		timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
	};

private:
	std::queue<Position> waypoint_queue;
	Position current_position;

	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_target_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _subscription;
	rclcpp::TimerBase::SharedPtr timer_;

	void update_waypoints();
	void send_current_target();
};


void Navigation::send_current_target() {
	Position curr_target = waypoint_queue.front();
	if (get_position_difference(curr_target, this->current_position) < this->get_parameter("equivalence_padding").as_double()) {
		waypoint_queue.pop();
		if (waypoint_queue.empty()) {
			return;
		}
		curr_target = waypoint_queue.front();
	}
	geometry_msgs::msg::Point msg;
	msg.x = curr_target.x;
	msg.y = curr_target.y;
	msg.z = curr_target.z;
	current_target_publisher_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "NAVIGATION: TargetPos(X=%f, Y=%f, Z=%f)", msg.x, msg.y, msg.z);
}

void Navigation::update_waypoints() {
	//TODO: This is temporary eventialy will be pathfinding and routing code
	if (waypoint_queue.empty()) {
		for (int i=0; i<10; i++) {
			Position pos;
			pos.x = i;
			pos.y = i * (-1 * (i%2));
			pos.z = -5.f;
			waypoint_queue.push(pos);
		}
	}
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Navigation>());
	rclcpp::shutdown();
	return 0;
}
