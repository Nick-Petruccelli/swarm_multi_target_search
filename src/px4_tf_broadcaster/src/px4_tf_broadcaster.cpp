#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

class Px4TFBroadcaster : public rclcpp::Node {
public:
	Px4TFBroadcaster() : rclcpp::Node("px4_tf_broadcaster") {
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		auto pose_recived_callback = [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
			geometry_msgs::msg::TransformStamped tf;
			tf.header.stamp = this->get_clock()->now();
			tf.header.frame_id = "odom";
			tf.child_frame_id = "base_link";
			tf.transform.translation.x = msg->position[0];
			tf.transform.translation.y = msg->position[1];
			tf.transform.translation.z = msg->position[2];
			tf.transform.rotation.x = msg->q[0];
			tf.transform.rotation.y = msg->q[1];
			tf.transform.rotation.z = msg->q[2];
			tf.transform.rotation.w = msg->q[3];
			tf_broadcaster_->sendTransform(tf);

			geometry_msgs::msg::TransformStamped t2;

			t2.header.stamp = this->get_clock()->now();
			t2.header.frame_id = "base_link";
			t2.child_frame_id = "lidar_link";

			// fixed offset of lidar on drone
			t2.transform.translation.x = 0.0;
			t2.transform.translation.y = 0.0;
			t2.transform.translation.z = 0.10;

			t2.transform.rotation.x = 0.0;
			t2.transform.rotation.y = 0.0;
			t2.transform.rotation.z = 0.0;
			t2.transform.rotation.w = 1.0;
			tf_broadcaster_->sendTransform(t2);	
		};
		px4_pose_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, pose_recived_callback);
	};

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pose_subscription_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4TFBroadcaster>());
	rclcpp::shutdown();
	return 0;
}
