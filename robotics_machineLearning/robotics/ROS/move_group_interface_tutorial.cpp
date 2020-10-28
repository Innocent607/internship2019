#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	/*
	______________________________________________________SETUP________________________________________________________________
	*/
	
	/* 
	MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  	   the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
	*/
	
	// it will retain its value till the execution of the program and also, it will not accept any change in its value
	static const std::string PLANNING_GROUP = "param_arm";
	
	// Setting move_group class; by specifying the name of planning group to plan and control for
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	
	/* 
	We will use the :planning_scene_interface:`PlanningSceneInterface`
       class to add and remove collision objects in our "virtual world" scene
    */
    moveit::planning_interface::MoveGroupInterface planning_scene_interface;
    
    /*
    Raw pointers are frequently used to refer to the planning group for improved performance.
    */
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState() -> getJointModelGroup(PLANNING_GROUP);
    
    /*
    _______________________________________________________VISUALIZATION______________________________________________________
    
    */
    
    // create two variables or member functions having the same name.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    
    // User remote control mode via buttons and keyboard
    visual_tools.loadRemoteControl();
    
    // RViz provides many types of markers, in this demo we will use text, cylinders, and sphere
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();
    
    /*
    ___________________________________________________BASIC____INFORMATION____________________________________________________
    
    */
    
    // printing name of the reference frame for the robot
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    
    // printing name of the end effector for the robot
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    /*
    ________________________________________________________START____DEMO______________________________________________________
    
    */
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolGUI window to start the demo");
    
    
    /*
    ___________________________________________________PLANNING__TO__A__POSE__GOAL______________________________________________
    
    */
    
    // plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);
    
    /*
    call the planner to compute the plan and visualize it
    N.B: its a plan but not asking move_group to actualy move the robot 
    */
    moveit::planning_interface::MoveGroupInterface::Plan my_Plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal)  %s", success ? "" : "FAILED");
    
    /*
    _______________________________________________________VISUALIZING____PLANS__________________________________________________
    
    */
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabelled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
    /*
    _______________________________________________________MOVING__TO__A__POSE__GOAL________________________________________________
    
    */
    
    /* Uncomment below line when working with a real robot */
	/* move_group.move(); */
    
    /*
    _________________________________________________PLANNING__TO__A__JOINT-SPACE__GOAL________________________________________________
    
    */
    
	/* Let's set a joint space goal and move towards it.  This will replace the pose target we set above.

  	   To start, we'll create an pointer that references the current robot's state. 
  	   RobotState is the object that contains all the current position/velocity/acceleration data.
  	*/
  	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  	
  	// get current set of joint values for the group
  	
  	std::vector<double> joint_group_positions;
  	current_state -> copyJointGroupPositions(joint_model_group, joint_group_positions);
  	
  	// modifying one of the joints, plan to the new joint space goal and visualize the plan.
  	joint_group_positions[0] = -1.0;  // radians
  	move_group.setJointValueTarget(joint_group_positions);
  	
  	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
	
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
	
    /*
    ______________________________________________PLANNING__WITH__PATH__CONSTRAINTS__________________________________________________
    
    */
    
    // specifing path constraints for a link on the robot.
    moveit_msgs::OrientationConstant ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    
    // setting it as the path constraint for the group.
    moveit_msgs::Constraints tests_contraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathContraints(test_constraints);
    
    //  setting the start state to a new pose.
    robot_state::RobotState start_state(*move_group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
	start_state.setFromIK(joint_model_group, start_pose2);
	move_group.setStartState(start_state);
    
    // Now we will plan to the earlier pose target from the new start state that we have just created.
    move_group.setPoseTarget(target_pose1);
    
    /*  since we call IK solver for planning joint angles for the desired target, this might be computationally intensive.
    	So increase the plan time from 5s to be sure the right pose is computed
    */
    move_group.setPlanningTime(10.0);
    
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
	
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishAxisLabeled(start_pose2, "start");
	visual_tools.publishAxisLabeled(target_pose1, "goal");
	visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step");
	
	// we claer the path after execution
	move_group.clearPathConstraints();
	
	//clear the start state also since we declared it before planning other paths
	move_group.setStartStateToCurrentState();
	
    /*
    ______________________________________________CARTESIAN____PATHS__________________________________________________
    
    */
    
    // list of waypoints for the end-effector to go through. 
    geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target_pose3);
	
	target_pose3.position.z -= 0.2;
	waypoints.push_back(target_pose3);  // down
	
	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);  // right
	
	target_pose3.position.z += 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3);  // up and left
	
	/* reduce the speed of the robot arm via a scaling factor of the maxiumum speed of each joint. Note this is not the speed of the end 			effector point.
	*/
	move_group.setMaxVelocityScalingFactor(0.1);
	
	/*
	We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in Cartesian 		translation. We will specify the jump threshold as 0.0, effectively disabling it. Warning - disabling the jump threshold while operating 		real hardware can cause large unpredictable motions of redundant joints and could be a safety issue.
	*/
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	
	for (std::size_t i = 0; i < waypoints.size(); ++i)
		visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
	
    /*
    __________________________ADDING/REMOVING__OBJECTS__AND ATTACHING/ DETACHING OBJECTS__________________________________________
    
    */	
    
    //Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	
	// The id of the object is used to identify it.
	collision_object.id = "box1";
	
	// Define a box to add to the world.
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.4;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.4;
	
	// Define a pose for the box (specified relative to frame_id)
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.4;
	box_pose.position.y = -0.2;
	box_pose.position.z = 1.0;
	
	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;
	
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	
	//Now, let’s add the collision object into the world
	ROS_INFO_NAMED("tutorial", "Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);
	
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	
	//Wait for MoveGroup to recieve and process the collision object message
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
	
	// Now when we plan a trajectory it will avoid the obstacle
	move_group.setStartState(*move_group.getCurrentState());
	geometry_msgs::Pose another_pose;
	another_pose.orientation.w = 1.0;
	another_pose.position.x = 0.4;
	another_pose.position.y = -0.4;
	another_pose.position.z = 0.9;
	move_group.setPoseTarget(another_pose);
	
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
	
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step");
	
	// Now, let’s attach the collision object to the robot.
	ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
	move_group.attachObject(collision_object.id);
	
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	
	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the " "robot");
	
	//detach the collision object from the robot.
	ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
	move_group.detachObject(collision_object.id);
	
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the " "robot");
	
	// remove the collision object from the world.
	ROS_INFO_NAMED("tutorial", "Remove the object from the world");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

	
	ros::shutdown();
	return 0;
	
}
