#include "precise_land.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <optional>

static const std::string kModeName = "ARUCO_PRECISION_LAND";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

Precise_Land::Precise_Land(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&Precise_Land::targetPoseCallback, this, std::placeholders::_1));

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
			   rclcpp::QoS(1).best_effort(), std::bind(&Precise_Land::vehicleLandDetectedCallback, this, std::placeholders::_1));


  	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);

}

void Precise_Land::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void Precise_Land::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

    // 1. 임시 태그 객체를 생성합니다.
    //    이제 ArucoTag에 사용자 정의 생성자가 있으므로, 기본 생성자가 호출되어
    //    position은 (NAN, NAN, NAN)으로 초기화됩니다.
    ArucoTag received_tag;

    // 2. 수신된 메시지의 값들을 객체 멤버에 직접 할당합니다.
    received_tag.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    received_tag.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    received_tag.timestamp = _node.now(); // 콜백이 호출된 현재 시간

    // 3. 월드 좌표계로 변환된 최종 결과를 _tag 멤버 변수에 저장합니다.
    _tag = getTagWorld(received_tag);
}


// 카메라 상대 위치에서 월드 좌표계로 변환과정
Precise_Land::ArucoTag Precise_Land::getTagWorld(const ArucoTag& tag)
{
	// Convert from optical to NED
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	Eigen::Matrix3d R;
	R << 0, -1, 0,
		 1, 0, 0,
		 0, 0, 1;
	// 이 회전변환을 짐벌락 현상없애기에 쿼터니언으로 바꿈
	Eigen::Quaterniond quat_NED(R);

	// 카메라에서 얼마만큼 이동해라 좌표생성
	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	// 카메라에서 얼마만큼 회전해라 쿼터니언 생성
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

    // 수정된 코드
    ArucoTag world_tag; // 기본 생성자가 호출되어 position이 (NAN, NAN, NAN)으로 초기화됨
    
    world_tag.position = tag_world_transform.translation();
    world_tag.orientation = Eigen::Quaterniond(tag_world_transform.rotation());
    world_tag.timestamp = tag.timestamp; // 입력으로 받은 태그의 타임스탬프를 그대로 전달

	return world_tag;
}




void Precise_Land::onActivate()
{
	_tag = ArucoTag();
	RCLCPP_INFO(_node.get_logger(),"Tag detected in Search state. Switching to Approach."
	" Tag Position (World NED): x=%.2f, y=%.2f, z=%.2f",
	_tag.position.x(), _tag.position.y(), _tag.position.z());
	generateSearchWaypoints();
	switchToState(State::Search);
}

void Precise_Land::onDeactivate()
{
	// No-op
}

void Precise_Land::updateSetpoint(float dt_s)
{
	bool target_lost = checkTargetTimeout();

	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired");
	}

	_target_lost_prev = target_lost;

	// State machine
	switch (_state) {
	case State::Idle: {
		// No-op -- just spin
		break;
	}
	case State::Search: {

		if (!std::isnan(_tag.position.x())) {

			RCLCPP_INFO(_node.get_logger(),"Tag detected in Search state. Switching to Approach."
			" Tag Position (World NED): x=%.2f, y=%.2f, z=%.2f",
			_tag.position.x(), _tag.position.y(), _tag.position.z());

			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		auto waypoint_position = _search_waypoints[_search_waypoint_index];

		_trajectory_setpoint->updatePosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;
			// If we have searched all waypoints, start over
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

	case State::Approach: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Approach using position setpoints
		auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);

		_trajectory_setpoint->updatePosition(target_position);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Descend using velocity setpoints and P velocity controller for XY
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));

		if (_land_detected) {
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f Precise_Land::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component  --목표지점까지의 상대 위치 오차
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// 0.1m/s min vel and 3m/s max vel
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

bool Precise_Land::checkTargetTimeout()
{
	if (!_tag.valid()) { return false; }

	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void Precise_Land::generateSearchWaypoints()
{
	// --- 수정된 코드 시작 ---
	RCLCPP_INFO(_node.get_logger(), "Generating continuous spiral search waypoints...");

	// The search waypoints are generated in the vehicle's local NED frame
	// starting from its current position.

	// Parameters for the continuous spiral search pattern
	// 이 값들을 조절하여 탐색 패턴을 변경할 수 있습니다.
	const double radius_increase_per_revolution = 4.0; // 360도 회전할 때마다 나선 반경이 증가하는 양 (미터)
	const double altitude_increase_per_revolution = 0.2; // 360도 회전할 때마다 하강하는 높이 (NED 좌표계에서는 양수)
	const int    points_per_revolution = 16;             // 한 바퀴를 구성하는 웨이포인트 수 (많을수록 부드러움)
	const int    num_revolutions = 10;                   // 생성할 총 회전 수 (탐색 범위 결정)

	std::vector<Eigen::Vector3f> waypoints;

	// Get the drone's current position to start the spiral from there
	const double start_x = _vehicle_local_position->positionNed().x();
	const double start_y = _vehicle_local_position->positionNed().y();
	const double start_z = _vehicle_local_position->positionNed().z();

	// Calculate increments per point for smooth transitions
	const double radius_increment_per_point = radius_increase_per_revolution / points_per_revolution;
	const double z_increment_per_point = altitude_increase_per_revolution / points_per_revolution;
	const double angle_increment_per_point = 2.0 * M_PI / points_per_revolution;

	const int total_points = num_revolutions * points_per_revolution;

	for (int i = 0; i <= total_points; ++i) {
		// Angle increases continuously
		const double angle = i * angle_increment_per_point;

		// Radius increases continuously and smoothly from the center
		const double radius = i * radius_increment_per_point;

		// Altitude decreases continuously and smoothly
		const double z = start_z + (i * z_increment_per_point);

		// Calculate waypoint coordinates relative to the starting position
		const double x = start_x + radius * cos(angle);
		const double y = start_y + radius * sin(angle);

		waypoints.push_back(Eigen::Vector3f(x, y, z));
	}

	_search_waypoints = waypoints;

	if (!_search_waypoints.empty()) {
		const auto& last_wp = _search_waypoints.back();
		RCLCPP_INFO(_node.get_logger(), "Generated %zu waypoints. Search ends at (x:%.1f, y:%.1f, z:%.1f)",
			_search_waypoints.size(), last_wp.x(), last_wp.y(), last_wp.z());
	} else {
		RCLCPP_WARN(_node.get_logger(), "Failed to generate search waypoints.");
	}
	// --- 수정된 코드 끝 ---
}

bool Precise_Land::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

std::string Precise_Land::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
	case State::Descend:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

void Precise_Land::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<Precise_Land>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
