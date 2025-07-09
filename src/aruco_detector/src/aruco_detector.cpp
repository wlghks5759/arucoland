#include "aruco_detector.hpp"
#include <sstream>

Aruco_DetectorNode::Aruco_DetectorNode()
	: Node("aruco_detector_node")
{
	RCLCPP_INFO(this->get_logger(), "Starting Aruco_DetectorNode");

	loadParameters();

	//ArUco 마커를 검출할 때 사용되는 알고리즘의 세부 설정값(파라미터)들을 정의하는 cv::aruco::DetectorParameters 구조체
	// See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
	auto detectorParams = cv::aruco::DetectorParameters();

	//아루코 마커 종류 ex) DICT_4X4_250, DICT_5X5_100 등
	// See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
	auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

	_detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);

	// RMW QoS settings
	auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

	// Subscribers
	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
			     "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", qos, std::bind(&Aruco_DetectorNode::image_callback, this, std::placeholders::_1));

	_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
				   "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info", qos, std::bind(&Aruco_DetectorNode::camera_info_callback, this, std::placeholders::_1));

	// Publishers
	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
			     "/image_proc", qos);
	_target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
				   "/target_pose", qos);
}

void Aruco_DetectorNode::loadParameters()
{
	declare_parameter<int>("aruco_id", 0);
	declare_parameter<int>("dictionary", 2); // DICT_4X4_250   id 0~249 인식할 수 있는 마커
	declare_parameter<double>("marker_size", 0.5);

	get_parameter("aruco_id", _param_aruco_id);
	get_parameter("dictionary", _param_dictionary);
	get_parameter("marker_size", _param_marker_size);
}

void Aruco_DetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		// Convert ROS image message to OpenCV image (Deep Copy)
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// Detect markers(ids 여러개인 것 가정)
		std::vector<int> ids;
		// 2차원(x,y) 꼭짓점4개 2자리 소숫점 float까지 --> 이거는 카메라 이미지 기준 xy픽셀임
		std::vector<std::vector<cv::Point2f>> corners;
		//마커 감지하라는 ...?
		_detector->detectMarkers(cv_ptr->image, corners, ids);
		//마커 모서리 그리기
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);


		if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
			// Calculate marker Pose and draw axes

			std::vector<std::vector<cv::Point2f>> undistortedCorners;

			for (const auto& corner : corners) {
				std::vector<cv::Point2f> undistortedCorner;
				//cornors의 왜곡된 좌표를 평평하게 보정
				cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
				undistortedCorners.push_back(undistortedCorner);
			}

			for (size_t i = 0; i < ids.size(); i++) {
				if (ids[i] != _param_aruco_id) {
					continue;
				}

				// 마커사이즈 미리알려주는 것. 이것을 통해 이미지 픽셀 분석하여 고도 추측 가능
				float half_size = _param_marker_size / 2.0f;
				std::vector<cv::Point3f> objectPoints = {
					cv::Point3f(-half_size,  half_size, 0),  // top left
					cv::Point3f(half_size,  half_size, 0),   // top right
					cv::Point3f(half_size, -half_size, 0),   // bottom right
					cv::Point3f(-half_size, -half_size, 0)   // bottom left
				};

				// Use PnP solver to estimate pose
				cv::Vec3d rvec, tvec;
				//이것이 핵심 함수 --> 마커크기를 가지고 고도가 몇인지 추측 (X,Y 도 알려줌)
				cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
				// 축그려줌 --> tvec: 마커의 위치, rvec: 마커의 회전(축-각 표현법)
				cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, _param_marker_size);
				// In OpenCV frame
				_target[0] = tvec[0];
				_target[1] = tvec[1];
				_target[2] = tvec[2];


				// Publish target pose
				geometry_msgs::msg::PoseStamped target_pose;
				target_pose.header.stamp = msg->header.stamp;
				target_pose.header.frame_id = "camera_frame"; // TODO: frame_id
				// Camera frame is RBU
				target_pose.pose.position.x = _target[0];
				target_pose.pose.position.y = _target[1];
				target_pose.pose.position.z = _target[2];
				//3*3행렬 담아두는 그릇
				cv::Mat rot_mat;
				//축각표현벡터를 3*3행렬로 바꿈(계산목적)
				cv::Rodrigues(rvec, rot_mat);
				RCLCPP_DEBUG(this->get_logger(), "Rot mat type: %d, rows: %d, cols: %d", rot_mat.type(), rot_mat.rows, rot_mat.cols);

				// Quaternion from rotation matrix
				if (rot_mat.type() == CV_64FC1 && rot_mat.rows == 3 && rot_mat.cols == 3) {
					//3*3 회전행렬을 쿼터니언으로 변환
					cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();
					target_pose.pose.orientation.x = quat.x;
					target_pose.pose.orientation.y = quat.y;
					target_pose.pose.orientation.z = quat.z;
					target_pose.pose.orientation.w = quat.w;
					
					//이게 아루코 마커 월드에서의 위치 및 쿼터니언 정보
					_target_pose_pub->publish(target_pose);

				} else {
					RCLCPP_ERROR(this->get_logger(), "Rotation matrix malformed!");
				}
				
			}

		} else {
			RCLCPP_ERROR(this->get_logger(), "distance to ground is NAN");
		}

		// Annotate the image
		annotate_image(cv_ptr);

		// Publish image
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = cv_ptr->image;
		_image_pub->publish(*out_msg.toImageMsg().get());

	} catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}
}

void Aruco_DetectorNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
		return;
	}

	// Always update the camera matrix and distortion coefficients from the new message
	//clone() 을통해 깊은복사해야함 안그러면 콜백끝나면 변수 사라지는데 그러면 이미지 콜백에서 이 변수 사용 못함
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(this->get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(this->get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(this->get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(this->get_logger(), "Updated camera intrinsics from camera_info topic.");
	}
}

void Aruco_DetectorNode::annotate_image(cv_bridge::CvImagePtr image)

{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << _target[0] << " Y: " << _target[1]  << " Z: " << _target[2];
	std::string text_xyz = stream.str();


	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Aruco_DetectorNode>());
	rclcpp::shutdown();
	return 0;
}
