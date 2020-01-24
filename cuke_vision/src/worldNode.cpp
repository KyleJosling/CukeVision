// ----------------------------------------------------------
// NAME: World Node
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
#include "cuke_vision/worldNode.hpp"

const double FINGER_MAX = 6400;

// TODO add this to some utility file
tf::Quaternion EulerZYZtoQuaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}


// Constructor
worldNode::worldNode() {
    
    // Initialize subscribers and publishers
    cukeSub = nH.subscribe(cukeTopic, 100, &worldNode::objectCallback, this);
    cObjPub = nH.advertise<moveit_msgs::CollisionObject>(cObjTopic, 10);
    aObjPub = nH.advertise<moveit_msgs::AttachedCollisionObject>(aObjTopic, 10);

    // Get robot type, check if robot is connected
    nH.param<std::string>("/robot_type", robotType, "m1n6s200");
    nH.param<bool>("/robot_connected", robotConnected, true);
    
    if (robotConnected == false) {
        ROS_INFO("Robot not connected");
    }

    // Create robot model
    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    robotModel = robotModelLoader.getModel();

    // Construt a planning scene that maintains a state of the world TODO
    // diff?
    planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();
    planningScene.reset(new planning_scene::PlanningScene(robotModel)); 
    // planningSceneMonitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // Initialize the move group interface and planning scene interface
    armGroupInterface = new moveit::planning_interface::MoveGroupInterface(armPlanningGroup);
    gripperGroupInterface = new moveit::planning_interface::MoveGroupInterface(gripperPlanningGroup);

    armGroupInterface->setEndEffectorLink(robotType + "_end_effector");
    

    // Finger action client
    fingerClient = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
       ("/" + robotType + "_driver/fingers_action/finger_positions", false);
   
    // Wait for finger action server to come up
    while(robotConnected && !fingerClient->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }

    ROS_INFO("Finger action server is up");
    
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", armGroupInterface->getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", armGroupInterface->getEndEffectorLink().c_str());

    // Set home position, convert to geometry_msgs for ease of use
    tf::Pose tempHomePose;
    tempHomePose.setOrigin(tf::Vector3(0.21486, -0.203351, 0.418802));
    tempHomePose.setRotation(tf::Quaternion(0.644305, 0.320657, 0.42346, 0.550211));
    tf::poseTFToMsg(tempHomePose, homePose); 

    // TEST function
    // addTable();
    defineCartesianPose();
    addTestObject();
    // gripperAction(0);
    // gripperAction(6400);
    // moveToGoal();
}

// World destructor
worldNode::~worldNode() {

    cObjPub.shutdown(); 
    aObjPub.shutdown(); 

    delete armGroupInterface;
    delete gripperGroupInterface;
    delete fingerClient;

}

void worldNode::addTestObject() {

     ROS_INFO("Adding cuke");

    //add target_cylinder
    cObj.id = "target_cylinder";
    cObj.header.frame_id = "world";

    // Define the primitive and add its dimensions
    cObj.primitives.resize(1);
    cObj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    cObj.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    cObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.25;
    cObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.015;

    // Define the pose of the object
    cObj.primitive_poses.resize(1);
    cObj.primitive_poses[0].position.x = 0.55;
    cObj.primitive_poses[0].position.y = 0.0;
    cObj.primitive_poses[0].position.z = 0.4;

    cObj.operation = moveit_msgs::CollisionObject::ADD;
    cObjPub.publish(cObj);

    std::vector<moveit_msgs::CollisionObject> cObjs;
    cObjs.push_back(cObj);
    planningSceneInterface->applyCollisionObject(cObj);
    pickCucumber(cObj);
}

// Prints the current pose of the robot
void worldNode::printRobotPose() {

    // Get current state of robot
    geometry_msgs::PoseStamped currentPose = armGroupInterface->getCurrentPose();

    std::cout << "Position (xyz) : " << 
        currentPose.pose.position.x << " " << 
        currentPose.pose.position.y << " " << 
        currentPose.pose.position.z << std::endl;

    std::cout << "Orientation (xyzw) : " <<
        currentPose.pose.orientation.x << " " <<
        currentPose.pose.orientation.y << " " <<
        currentPose.pose.orientation.z << " " <<
        currentPose.pose.orientation.w << std::endl;
}

// Define preset cartesian poses
void worldNode::defineCartesianPose() {

    tf::Quaternion q;

    // Grasp pose
    graspPose.header.frame_id = "world";
    graspPose.header.stamp = ros::Time::now();

    // EULER ZYZ (-pi/4, pi/2, pi/2)
    // Dummy numbers for position since position is defined by cucumber
    // location
    graspPose.pose.position.x = 0.0;
    graspPose.pose.position.y = 0.5;
    graspPose.pose.position.z = 0.45;

    q = EulerZYZtoQuaternion(0, M_PI/2, M_PI/2);
    graspPose.pose.orientation.x = q.x(); // TODO could make this one line
    graspPose.pose.orientation.y = q.y();
    graspPose.pose.orientation.z = q.z();
    graspPose.pose.orientation.w = q.w();

    // Place pose
    placePose.header.frame_id = "world";
    placePose.header.stamp = ros::Time::now();

    // EULER ZYZ (-pi/4, pi/2, pi/2)
    // Dummy numbers for position since position is defined by cucumber
    // location
    placePose.pose.position.x = -0.2;
    placePose.pose.position.y = 0.4;
    placePose.pose.position.z = 0.4;

    q = EulerZYZtoQuaternion(0, M_PI/2, M_PI/2);
    placePose.pose.orientation.x = q.x(); // TODO could make this one line
    placePose.pose.orientation.y = q.y();
    placePose.pose.orientation.z = q.z();
    placePose.pose.orientation.w = q.w();
}

// Closes or open gripper 
void worldNode::gripperAction(bool open, trajectory_msgs::JointTrajectory &posture) {
    
    posture.joint_names.resize(2);
    posture.joint_names[0] = "m1n6s200_joint_finger_1";
    posture.joint_names[1] = "m1n6s200_joint_finger_2";
    
    double pos;
    pos = open ? 0.0 : 1.2;
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = pos;
    posture.points[0].positions[1] = pos;
    posture.points[0].time_from_start = ros::Duration(0.5);

}

// TODO test function
void worldNode::moveToGoal() {
    
    // Set the target pose
    armGroupInterface->setApproximateJointValueTarget(placePose, "m1n6s200_link_6");

    // TODO make member variable?
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (armGroupInterface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("planning successful ? : %d ", success);

    // armGroupInterface->execute(plan);
    armGroupInterface->move();

    // // // Set the target pose
    // armGroupInterface->setNamedTarget("Home");

    // success = (armGroupInterface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO("planning successful ? : %d ", success);

    // armGroupInterface->execute(plan);
}

// Receives a new cucumber
void worldNode::objectCallback(const moveit_msgs::CollisionObject &objectMsg) {
    
    ROS_INFO("Cuke has been received with height: %f and radius: %f",
        objectMsg.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT],
        objectMsg.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);

    cObjPub.publish(objectMsg);

    planningSceneInterface->applyCollisionObject(objectMsg);
    ROS_INFO("Cuke has been applied.");
}

// Removes a collision object from the world
void worldNode::removeCucumber() {

}

// Pick cucumber
void worldNode::pickCucumber(const moveit_msgs::CollisionObject &cucumber) {
    
    // Pop cucumber from stack
    // cObj = cucumbers.pop();

    std::vector<moveit_msgs::Grasp> grasps;
    tf::Quaternion q; // TODO consider switching to tf2 
    
    // Only one grasp for now
    grasps.resize(1);
    
    // Grasp pose (same as already set really)
    grasps[0].grasp_pose.header.frame_id = "world";
    q = EulerZYZtoQuaternion(0, M_PI/2, M_PI/2);
    tf::quaternionTFToMsg(q, grasps[0].grasp_pose.pose.orientation);
    grasps[0].grasp_pose.pose.position.x = 0.55;
    grasps[0].grasp_pose.pose.position.y = 0.0;
    grasps[0].grasp_pose.pose.position.z = 0.4;

    // Pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "m1n6s200_end_effector"; // TODO make constant
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0; // Approach in Z direction of end effector
    grasps[0].pre_grasp_approach.min_distance = 0.07;
    grasps[0].pre_grasp_approach.desired_distance = 0.10;
    
    grasps[0].post_grasp_retreat.direction.header.frame_id = "m1n6s200_end_effector"; 
    grasps[0].post_grasp_retreat.direction.vector.z = -1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.07;
    grasps[0].post_grasp_retreat.desired_distance = 0.10;

    // Open and close gripper when the time strikes 12 and the planets are aligned
    gripperAction( true,  grasps[0].pre_grasp_posture);
    gripperAction( false, grasps[0].grasp_posture);

    armGroupInterface->pick("target_cylinder", grasps);

    // Set tolerances (TODO TEST)
    armGroupInterface->setGoalPositionTolerance(0.03);
    armGroupInterface->setGoalOrientationTolerance(0.26);

    moveToGoal(); // TODO test function
}

int main (int argc, char** argv ) {
    
    std::string nodeName = "worldNode";
    ros::init(argc, argv, nodeName);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    worldNode w;

    ros::waitForShutdown();
    return 0;
}
