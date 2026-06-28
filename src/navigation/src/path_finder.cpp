#include <atomic>
#include <cassert>
#include <cmath>
#include <cstring>
#include <queue>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <unordered_set>

#define OCCUPANCY_GRID_WIDTH 300
#define OCCUPANCY_GRID_HEIGHT 50
#define OCCUPANCY_GRID_CELL_SIZE .5

#define COLLISION_CHECK_STEP_LENGTH .1
#define COLLISION_BOX_HALF_LENGTH .25f

#define CALCULATE_PATH_MAX_STEPS 5000
#define PATH_FINDING_MOVE_STEP_LENGTH .5

struct Position {
	float x=0;
	float y=0;
	float z=0;
};

struct Orientation {
	float x=0;
	float y=0;
	float z=0;
	float w=0;
};


struct GridCord {
	int x;
	int y;
	int z;

	bool operator==(const GridCord &other) const {
		return x == other.x &&
		       y == other.y &&
		       z == other.z;
	}
};

struct GridCordHash {
    size_t operator()(const GridCord &g) const noexcept {
        size_t h1 = std::hash<int>()(g.x);
        size_t h2 = std::hash<int>()(g.y);
        size_t h3 = std::hash<int>()(g.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct PathNode {
	GridCord position;
	float distance;
	float huristic;
	std::shared_ptr<PathNode> prev_node;
};

bool grid_cord_is_equal(const GridCord &cord1, const GridCord &cord2) {
	return cord1.x == cord2.x && cord1.y == cord2.y && cord1.z == cord2.z;
}

struct PathNodeComp {
	bool operator()(const std::shared_ptr<PathNode> &left, const std::shared_ptr<PathNode> &right) {
		float f_left = left->distance + left->huristic;
		float f_right = right->distance + right->huristic;
		return f_left > f_right;
	}
};

auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

class PathFinder : public rclcpp::Node {
public:
	PathFinder() : rclcpp::Node("pathfinder") {
		auto current_target_callback = [this](geometry_msgs::msg::Point target) {
			cur_target = {(float)target.x, (float)target.y, (float)target.z};
			fill_occupancy_grid();
			calculate_path(cur_target);
		};
		current_target_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/navigation/current_target", qos, current_target_callback);
		
		auto lidar_callback = [this](sensor_msgs::msg::LaserScan::SharedPtr data) {
			if (reading_lidar_data)
				return;
			lidar_data = data;
		};
		lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", qos, lidar_callback);

		auto odom_callback = [this](px4_msgs::msg::VehicleOdometry odom_data) {
			cur_position = {
				odom_data.position[0],
				odom_data.position[1],
				odom_data.position[2]
			};
			cur_orientation = {
				odom_data.q[1],
				odom_data.q[2],
				odom_data.q[3],
				odom_data.q[0]
			};
		};
		odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, odom_callback);

		next_path_node_position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/path_finder/next_path_node", qos);
		auto path_update_timer_callback = [this]() -> void {
			fill_occupancy_grid();
			if(!is_current_path_interupted())
				return;
			calculate_path(cur_target);
		};
		path_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), path_update_timer_callback);
	};

private:
	std::vector<Position> current_path;
	int current_path_idx = 0;
	bool occupancy_grid[OCCUPANCY_GRID_WIDTH][OCCUPANCY_GRID_WIDTH][OCCUPANCY_GRID_HEIGHT] = {0}; // [x][y][z]
	std::atomic_bool reading_lidar_data = false;
	sensor_msgs::msg::LaserScan::SharedPtr lidar_data;
	Position cur_position;
	Orientation cur_orientation;
	Position cur_target;

	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr current_target_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscription_;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr next_path_node_position_publisher_;
	rclcpp::TimerBase::SharedPtr path_update_timer_;

	void fill_occupancy_grid();
	bool is_current_path_interupted();
	void calculate_path(Position target_pos);

	Position world_pos_to_grid_pos(const Position &world_pos);
	Position grid_pos_to_world_pos(const Position &world_pos);
	float get_position_distance(const Position &pos1, const Position &pos2);

	std::array<std::shared_ptr<PathNode>, 26> get_available_moves(const std::shared_ptr<PathNode> &node);
	std::vector<Position> backtrace_path(std::shared_ptr<PathNode> end);

	bool check_collisions_on_line(const Position &start, const Position &end);
	std::array<Position, 8> get_collision_box_corners(const Position &pos);
};

void PathFinder::calculate_path(Position target_pos) {
	RCLCPP_INFO(this->get_logger(), "Hit calc path");
	current_path.clear();
	Position grid_curr_pos = world_pos_to_grid_pos(cur_position);
	Position grid_target_pos = world_pos_to_grid_pos(target_pos);
	RCLCPP_INFO(this->get_logger(), "CurPos: (%f,%f,%f), TargetPos: (%f,%f,%f)", grid_curr_pos.x, grid_curr_pos.y, grid_curr_pos.z, grid_target_pos.x, grid_target_pos.y, grid_target_pos.z);
	GridCord start_grid_cord = {
		(int)(grid_curr_pos.x / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
		(int)(grid_curr_pos.y / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
		(int)(grid_curr_pos.z / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_HEIGHT / 2)
	};
	std::shared_ptr<PathNode> start_node = std::make_shared<PathNode>(PathNode {
		start_grid_cord,
		0.0,
		get_position_distance(grid_curr_pos, grid_target_pos),
		nullptr
	});
	std::priority_queue<std::shared_ptr<PathNode>, std::vector<std::shared_ptr<PathNode>>, PathNodeComp> frontier;
	std::unordered_set<GridCord, GridCordHash> visited;
	frontier.push(start_node);
	GridCord target_pos_grid_cord = {
		(int)(grid_target_pos.x / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
		(int)(grid_target_pos.y / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
		(int)(grid_target_pos.z / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_HEIGHT / 2)
	};
	RCLCPP_INFO(this->get_logger(), "TargetGridCord: (%d,%d,%d)", target_pos_grid_cord.x, target_pos_grid_cord.y, target_pos_grid_cord.z);
	std::vector<Position> path;
	for (int _=0; _<CALCULATE_PATH_MAX_STEPS; _++) {
		if (frontier.empty())
			return;
		auto node = frontier.top();
		frontier.pop();
		GridCord node_grid_cord = {
			(int)(node->position.x / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
			(int)(node->position.y / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
			(int)(node->position.z / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_HEIGHT / 2)
		};
		visited.insert(node_grid_cord);
		RCLCPP_INFO(this->get_logger(), "NodeGridCord: (%d,%d,%d)", node_grid_cord.x, node_grid_cord.y, node_grid_cord.z);
		std::array<std::shared_ptr<PathNode>, 26> available_moves = get_available_moves(node);
		for (size_t i=0; i<22; i++) {
			auto move = available_moves[i];
			move->prev_node = node;
			move->distance = node->distance + PATH_FINDING_MOVE_STEP_LENGTH;
			move->huristic = get_cord_distance(move->position, target_pos_grid_cord);
			GridCord grid_cord = {
				(int)(move->position.x / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
				(int)(move->position.y / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2),
				(int)(move->position.z / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_HEIGHT / 2)
			};
			if (visited.find(grid_cord) != visited.end())
				continue;
			if (occupancy_grid[grid_cord.x][grid_cord.y][grid_cord.z])
				continue;
			if (grid_cord_is_equal(grid_cord, target_pos_grid_cord)) {
				RCLCPP_INFO(this->get_logger(), "Hit end path");
				path = backtrace_path(move);
				break;
			}
			frontier.push(move);
		}
	}
	current_path.reserve(path.size());
	for (Position pos : path) {
		RCLCPP_INFO(this->get_logger(), "PathPoint: (%f,%f,%f)", pos.x, pos.y, pos.z);
		current_path.push_back(grid_pos_to_world_pos(pos));
	}
}

inline std::array<std::shared_ptr<PathNode>, 26> PathFinder::get_available_moves(const std::shared_ptr<PathNode> &node) {
	std::array<std::shared_ptr<PathNode>, 26> out;
	int out_idx = 0;
	float start_x = node->position.x;
	float start_y = node->position.y;
	float start_z = node->position.z;
	/*
	for (int x=-1; x<=1; x++) {
		for (int y=-1; y<=1; y++) {
			for (int z=-1; z<=1; z++) {
				if(x==0 && y==0 && z==0)
					continue;
				float magnitude = std::hypot(x,y,z);
				float dist = PATH_FINDING_MOVE_STEP_LENGTH;
				auto move = std::make_shared<PathNode>();
				move->position = {dist*(x/magnitude) + start_x,dist*(y/magnitude) + start_y,dist*(z/magnitude) + start_z};
				out[out_idx++] = move; 
			}
		}
	}
	*/
	return out;
}

std::vector<Position> PathFinder::backtrace_path(std::shared_ptr<PathNode> end_node) {
	std::queue<Position> pos_queue;
	auto cur_node = end_node;
	while (cur_node->prev_node != nullptr) {
		pos_queue.push(cur_node->position);
		cur_node = cur_node->prev_node;
	}
	std::vector<Position> out;
	out.reserve(pos_queue.size());
	while (!pos_queue.empty()) {
		Position pos = pos_queue.front();
		pos_queue.pop();
		out.push_back(pos);
	}
	return out;
}

void PathFinder::fill_occupancy_grid() {
	if (!lidar_data)
		return;
	memset(occupancy_grid, 0, sizeof(occupancy_grid));
	reading_lidar_data = true;
	const auto &ranges = lidar_data->ranges;
	size_t num_readings = ranges.size();
	float start_angle = lidar_data->angle_min;
	float angle_increment = lidar_data->angle_increment;
	reading_lidar_data = false;
	std::vector<Position> points;
	points.reserve(num_readings);
	size_t points_count = 0;
	for (size_t i=0; i<num_readings; i++) {
		float range = ranges[i];
		if (std::isinf(range) || std::isnan(range)) {
			continue;
		}
		float cur_angel = start_angle + (i * angle_increment);
		Position point;
		point.x = range * cos(cur_angel);
		point.y = range * sin(cur_angel);
		point.z = cur_position.z;
		points.push_back(point);
		points_count++;
	}
	for (size_t i=0; i<points_count; i++) {
		Position point = points[i];
		//RCLCPP_INFO(this->get_logger(), "LIDAR_POINT: (%f,%f,%f)", point.x, point.y, point.z);
		int occupancy_grid_x = (int)(point.x / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2);
		int occupancy_grid_y = (int)(point.y / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2);
		//RCLCPP_INFO(this->get_logger(), "LIDAR_POINT: (%d,%d,Z)", occupancy_grid_x, occupancy_grid_y);
		if (occupancy_grid_x >= OCCUPANCY_GRID_WIDTH)
			continue;
		if (occupancy_grid_x < 0)
			continue;
		if (occupancy_grid_y >= OCCUPANCY_GRID_WIDTH)
			continue;
		if (occupancy_grid_y < 0)
			continue;
		for (int j=0; j<OCCUPANCY_GRID_HEIGHT; j++) {
			occupancy_grid[occupancy_grid_x][occupancy_grid_y][j] = true;
		}
	}
};

bool PathFinder::is_current_path_interupted() {
	if(current_path.size() < 1) {
		RCLCPP_INFO(this->get_logger(), "Hit empty path check in interupt check");
		return true;
	}
	Position start_pos = cur_position;
	for (size_t i=current_path_idx; i<current_path.size(); i++) {
		Position end_pos = current_path[i];
		bool collision = check_collisions_on_line(start_pos, end_pos);
		if (collision)
			return true;
		start_pos = end_pos;
	}
	return false;
}

bool PathFinder::check_collisions_on_line(const Position &start_pos, const Position &end_pos) {
	Position grid_start_pos = world_pos_to_grid_pos(start_pos);
	Position grid_end_pos = world_pos_to_grid_pos(end_pos);
	std::array<Position, 8> corner_positions = get_collision_box_corners(grid_start_pos);
	float distance = get_position_distance(grid_start_pos, grid_end_pos);
	Position start_end_differance = {
		grid_end_pos.x - grid_start_pos.x,
		grid_end_pos.y - grid_start_pos.y,
		grid_end_pos.z - grid_start_pos.z
	};
	float start_end_differance_magnitude = std::hypot(start_end_differance.x, start_end_differance.y, start_end_differance.z);
	Position step_vector = {
		start_end_differance.x / start_end_differance_magnitude,
		start_end_differance.y / start_end_differance_magnitude,
		start_end_differance.z / start_end_differance_magnitude,
	};
	while(distance > COLLISION_CHECK_STEP_LENGTH) {
		for (Position corner_pos : corner_positions){
			corner_pos.x += step_vector.x;
			corner_pos.y += step_vector.y;
			corner_pos.z += step_vector.z;
			int occupancy_grid_x = (corner_pos.x / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2);
			int occupancy_grid_y = (corner_pos.y / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_WIDTH / 2);
			int occupancy_grid_z = (corner_pos.z / OCCUPANCY_GRID_CELL_SIZE) + (OCCUPANCY_GRID_HEIGHT / 2);
			if (occupancy_grid_x > OCCUPANCY_GRID_WIDTH)
				continue;
			if (occupancy_grid_x < 0)
				continue;
			if (occupancy_grid_y > OCCUPANCY_GRID_WIDTH)
				continue;
			if (occupancy_grid_y < 0)
				continue;
			if (occupancy_grid_z > OCCUPANCY_GRID_HEIGHT)
				continue;
			if (occupancy_grid_z < 0)
				continue;
			if (occupancy_grid[occupancy_grid_x][occupancy_grid_y][occupancy_grid_z])
				return true;
		}
		distance -= COLLISION_CHECK_STEP_LENGTH;
	}
	return false;
}

inline std::array<Position, 8> PathFinder::get_collision_box_corners(const Position &pos) {
	std::array<Position, 8> corners;
	corners[0] = {pos.x + COLLISION_BOX_HALF_LENGTH, pos.y - COLLISION_BOX_HALF_LENGTH, pos.z + COLLISION_BOX_HALF_LENGTH}; //front left top
	corners[1] = {pos.x + COLLISION_BOX_HALF_LENGTH, pos.y + COLLISION_BOX_HALF_LENGTH, pos.z + COLLISION_BOX_HALF_LENGTH}; //front right top
	corners[2] = {pos.x - COLLISION_BOX_HALF_LENGTH, pos.y - COLLISION_BOX_HALF_LENGTH, pos.z + COLLISION_BOX_HALF_LENGTH}; //back right top
	corners[3] = {pos.x - COLLISION_BOX_HALF_LENGTH, pos.y + COLLISION_BOX_HALF_LENGTH, pos.z + COLLISION_BOX_HALF_LENGTH}; //back left top
	corners[4] = {pos.x + COLLISION_BOX_HALF_LENGTH, pos.y - COLLISION_BOX_HALF_LENGTH, pos.z - COLLISION_BOX_HALF_LENGTH}; //front left bot
	corners[5] = {pos.x + COLLISION_BOX_HALF_LENGTH, pos.y + COLLISION_BOX_HALF_LENGTH, pos.z - COLLISION_BOX_HALF_LENGTH}; //front right bot
	corners[6] = {pos.x - COLLISION_BOX_HALF_LENGTH, pos.y + COLLISION_BOX_HALF_LENGTH, pos.z + COLLISION_BOX_HALF_LENGTH}; //back right bot
	corners[7] = {pos.x - COLLISION_BOX_HALF_LENGTH, pos.y + COLLISION_BOX_HALF_LENGTH, pos.z + COLLISION_BOX_HALF_LENGTH}; //back left bot 
	return corners;
}

inline Position PathFinder::world_pos_to_grid_pos(const Position &world_pos) {
	Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
	Eigen::Quaternionf q(
		cur_orientation.w,
		cur_orientation.x,
		cur_orientation.y,
		cur_orientation.z
	);
	transform.linear() = q.toRotationMatrix();
	transform.translation() << cur_position.x, cur_position.y, cur_position.z;
	Eigen::Vector3f world_vec(world_pos.x, world_pos.y, world_pos.z);
	Eigen::Vector3f grid_vec = transform.inverse() * world_vec;
	return {grid_vec.x(), grid_vec.y(), grid_vec.z()};
}

inline Position PathFinder::grid_pos_to_world_pos(const Position &grid_pos) {
	Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
	Eigen::Quaternionf q(
		cur_orientation.w,
		cur_orientation.x,
		cur_orientation.y,
		cur_orientation.z
	);
	transform.linear() = q.toRotationMatrix();
	transform.translation() << cur_position.x, cur_position.y, cur_position.z;
	Eigen::Vector3f grid_vec(grid_pos.x, grid_pos.y, grid_pos.z);
	Eigen::Vector3f world_vec = transform * grid_vec;
	return {world_vec.x(), world_vec.y(), world_vec.z()};
}

inline float PathFinder::get_position_distance(const Position &pos1, const Position &pos2) {
	return std::hypot(
		pos2.x - pos1.x,
		pos2.y - pos1.y,
		pos2.z - pos1.z
	);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PathFinder>());
	rclcpp::shutdown();
	return 0;
}
