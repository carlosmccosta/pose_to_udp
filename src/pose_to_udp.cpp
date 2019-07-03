/**\file pose_to_udp.cpp
 * \brief PoseToUDP implementation
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <pose_to_udp/pose_to_udp.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace pose_to_udp {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PoseToUDP::PoseToUDP() :
	udp_port_(1337),
	send_6dof_pose_in_udp_message_(false),
	send_quaternion_in_udp_message_(false),
	udp_file_descriptor_(-1),
	tf_buffer_(ros::Duration(20)),
	tf_listener_(tf_buffer_),
	tf_lookup_timeout_(0.2)
	{}


PoseToUDP::~PoseToUDP() {
	if (udp_file_descriptor_ >= 0) {
		close(udp_file_descriptor_);
	}
}


void PoseToUDP::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;

	private_node_handle_->param("pose_topic", pose_topic_, std::string("pose"));
	private_node_handle_->param("udp_group_address", udp_group_address_, std::string(""));
	private_node_handle_->param("udp_port", udp_port_, 1337);
	private_node_handle_->param("send_6dof_pose_in_udp_message", send_6dof_pose_in_udp_message_, false);
	private_node_handle_->param("send_quaternion_in_udp_message", send_quaternion_in_udp_message_, false);
	private_node_handle_->param("udp_pose_coordinate_system", udp_pose_coordinate_system_, std::string("conveyor"));
	private_node_handle_->param("tf_lookup_timeout", tf_lookup_timeout_, 0.2);
	private_node_handle_->param("show_in_console_the_udp_message_data", show_in_console_the_udp_message_data_, false);
}


void PoseToUDP::setupUDP() {
	if (udp_port_ <= 0) {
		ROS_ERROR("The udp_port must be > 0");
		exit(EXIT_FAILURE);
	}

	udp_file_descriptor_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_file_descriptor_ < 0) { 
		ROS_ERROR("Failed to create socket file discriptor");
		exit(EXIT_FAILURE);
	}

	memset(&udp_server_address_, 0, sizeof(udp_server_address_));
	udp_server_address_.sin_family = AF_INET;
	udp_server_address_.sin_port = htons(udp_port_);
	if (udp_group_address_.empty()) {
		udp_server_address_.sin_addr.s_addr = htonl(INADDR_ANY);
	} else {
		udp_server_address_.sin_addr.s_addr = inet_addr(udp_group_address_.c_str());
	}
}


void PoseToUDP::setupROS() {
	ros::Time::waitForValid();
	pose_subscriber_ = node_handle_->subscribe(pose_topic_, 5, &pose_to_udp::PoseToUDP::poseCallback, this);
}


void PoseToUDP::start() {
	setupUDP();
	setupROS();
	ros::spin();
}


void PoseToUDP::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
	tf2::Transform pose;
	tf2::fromMsg(pose_msg->pose, pose);
	if (!udp_pose_coordinate_system_.empty() && udp_pose_coordinate_system_ != pose_msg->header.frame_id) {
		try{
			geometry_msgs::TransformStamped transform_stamped =
				tf_buffer_.lookupTransform(udp_pose_coordinate_system_, pose_msg->header.frame_id,
				pose_msg->header.stamp, ros::Duration(tf_lookup_timeout_));
			tf2::Transform transform;
			tf2::fromMsg(transform_stamped.transform, transform);
			pose.mult(transform, pose);
		} catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
		}
	}

	tf2::Vector3 origin = pose.getOrigin();
	tf2::Quaternion rotation = pose.getRotation();
	tf2::Matrix3x3 rotation_matrix(rotation);
	double yaw, pitch, roll;
	rotation_matrix.getEulerYPR(yaw, pitch, roll);

	std::vector<float> message;
	message.push_back((float)origin.x());
	message.push_back((float)origin.y());
	if (send_6dof_pose_in_udp_message_) {
		message.push_back((float)origin.z());
		if (send_quaternion_in_udp_message_) {
			message.push_back((float)rotation.x());
			message.push_back((float)rotation.y());
			message.push_back((float)rotation.z());
			message.push_back((float)rotation.w());
		} else {
			message.push_back((float)yaw);
			message.push_back((float)pitch);
			message.push_back((float)roll);
		}
	} else {
		message.push_back((float)yaw);
	}

	if (show_in_console_the_udp_message_data_) {
		std::stringstream ss;
		for (size_t i = 0; i < message.size(); ++i) {
			ss << message[i];
			if (i < message.size() - 1)
				ss << " | ";
		}
		ROS_INFO_STREAM("Sending UDP packet to [" << udp_group_address_ << ":" << udp_port_ << "] with data [ " << ss.str() << " ]");
	}

	size_t message_size = message.size() * sizeof(float);
	ssize_t number_butes_sent = sendto(udp_file_descriptor_, (const char *)message.data(), message_size,
		MSG_CONFIRM, (const struct sockaddr *) &udp_server_address_, sizeof(udp_server_address_));

	if (number_butes_sent < 0) {
		ROS_WARN_STREAM("Failed to send the UDP message with error code [ "
			<< errno << " -> " << strerror(errno) << " ]");
	} else if ((size_t)number_butes_sent < message_size) {
		ROS_WARN_STREAM("Failed to send the complete UDP message (sent "
			<< number_butes_sent << " bytes from a message with " << message_size << " bytes)");
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

} /* namespace pose_to_udp */
