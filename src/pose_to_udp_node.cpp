/**\file pose_to_udp_node.cpp
 * \brief Main for pose_to_udp::PoseToUDP
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <ros/ros.h>
#include <pose_to_udp/pose_to_udp.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_to_udp_node");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	pose_to_udp::PoseToUDP pose_to_udp;
	pose_to_udp.setupConfigurationFromParameterServer(node_handle, private_node_handle);
	pose_to_udp.start();

	return 0;
}
// ###################################################################################   </main>   #############################################################################
