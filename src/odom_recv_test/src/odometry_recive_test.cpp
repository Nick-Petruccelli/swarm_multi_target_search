#include <memory>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

class OdomRecvTest : public rclcpp::Node {
public:
	OdomRecvTest() : Node("odom_recv_test") {
		auto topic_callback = [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg){
			float x = msg->position[0];
			float y = msg->position[1];
			float z = msg->position[2];
			RCLCPP_INFO(this->get_logger(), "Recived Odom:\n %f, %f, %f", x, y, z);
		};
		_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, topic_callback);
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _subscription;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomRecvTest>());
	rclcpp::shutdown();
	return 0;
}
