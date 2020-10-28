
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	/*
	We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS 
	parameter server and construct a RobotModel for us to use.
	*/
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	
	/*
	Using the RobotModel, we can construct a RobotState that maintains the configuration of the robot. We will set all 
	joints in the state to their default values. We can then get a JointModelGroup, which represents the robot model for
	a particular group, e.g. the “panda_arm” of the Panda robot.
	*/
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
	
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	
	/*______________________________________________Get Joint Values________________________________________________________*/
	
	/*Here we get the current joint values stored in the state for the panda arm*/
	std::Vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); i++)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}
	
	/*______________________________________________Joint Limits________________________________________________________*/
	
	/* 	setJointGroupPositions() does not enforce joint limits by itself, 
		but a call to enforceBounds() will do it.
	*/
	/* Set one joint in the Panda arm outside its joint limit */
	joint_values[0] = 5.57;
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
	
	/* Check whether any joint is outside its joint limits */
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	
	/* Enforce the joint limits for this state and check again*/
	kinematic_state->enforceBounds();
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

	/*______________________________________________Forward Kinematics________________________________________________________*/
	
	/* 	
	we compute forward kinematics for a set of random joint values. 
	*/
	kinematic_state->setToRandomPositions(joint_model_group);
	const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");
	
	/* Print end-effector pose. Remember that this is in the model frame */
	ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
	ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
	
	/*______________________________________________Inverse Kinematics________________________________________________________*/
	
	/* 	
	to compute inverse kinematics we need:
		-  The desired pose of the end-effector (by default, this is the last link in the “panda_arm” chain)
		- The number of attempts to be made at solving IK: 10
		- The timeout for each attempt: 0.1 s
	*/
	std::size_t attempts = 10;
	double timeout = 0.1;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);
	
	/*we print out the IK solution (if found):*/
	if (found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}
	}
	
	else
	{
		ROS_INFO("Did not find IK solution");
	}
	
	/*______________________________________________Get the Jacobian________________________________________________________*/
	
	/* 	
	We will get the Jacobian from the RobotState. 
	*/
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
	kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
	ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
