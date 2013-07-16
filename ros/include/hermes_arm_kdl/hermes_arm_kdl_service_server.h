#include "hermes_arm_kdl/Fkine.h"
#include "ros/ros.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
//#include <hermes_grasp_service/HermesGrasp.h>
//#include <hermes_grasp_service/graspHand.h>

class HermesArmKdlServiceServer
{
public:
	HermesArmKdlServiceServer(ros::NodeHandle nh);
	void init();
	bool fkine(hermes_arm_kdl::Fkine::Request &req, hermes_arm_kdl::Fkine::Response &res);

protected:
	ros::NodeHandle node_;
	ros::ServiceServer hermes_arm_kdl_service_server_; 

	//GraspHand left_hand_;
	//GraspHand right_hand_;
};
