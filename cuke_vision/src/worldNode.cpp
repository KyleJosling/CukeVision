// ----------------------------------------------------------
// NAME: World Node
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
#include "cuke_vision/worldNode.hpp"

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
    
    // Subscriber to cucumber objects
    cukeSub = nH.subscribe(cukeTopic, 100, &worldNode::objectCallback, this);

    // Object publishers
    cObjPub = nH.advertise<moveit_msgs::CollisionObject>(cObjTopic, 10);
    aObjPub = nH.advertise<moveit_msgs::AttachedCollisionObject>(aObjTopic, 10);

    // stepper controller publisher
    positionControlPub = nH.advertise<std_msgs::Float32>(positionControlTopic, 10);


    // Get robot type, check if robot is connected
    nH.param<std::string>("/robot_type", robotType, "m1n6s200");
    nH.param<bool>("/robot_connected", robotConnected, true);
    
    if (robotConnected == false) {
        ROS_INFO("Robot not connected");
    }

    // Create robot model
    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    robotModel = robotModelLoader.getModel();

    // Construt a planning scene that maintains a state of the world
    planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();

    // Initialize the move group interface and planning scene interface
    armGroupInterface = new moveit::planning_interface::MoveGroupInterface(armPlanningGroup);
    gripperGroupInterface = new moveit::planning_interface::MoveGroupInterface(gripperPlanningGroup);

    armGroupInterface->setEndEffectorLink(robotType + "_end_effector");
    

    // Finger action client
    fingerClient = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
       ("/" + robotType + "_driver/fingers_action/finger_positions", false);
   
    // Wait for finger action server to come up
    while(robotConnected && !fingerClient->waitForServer(ros::Duration(5.0))){
      ROS_INFO("WORLD NODE : Waiting for the finger action server to come up");
    }

    ROS_INFO("WORLD NODE: Finger action server is up!");
    
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", armGroupInterface->getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", armGroupInterface->getEndEffectorLink().c_str());

    defineCartesianPose();
}

// World destructor
worldNode::~worldNode() {

    cObjPub.shutdown(); 
    aObjPub.shutdown(); 

    delete armGroupInterface;
    delete gripperGroupInterface;
    delete fingerClient;

}

// Test function for loading in cucumbers from yaml file
void worldNode::loadCucumbersFromFile() {
    testCucumbers.resize(3); 
    nH.getParam("cukeX", testCucumbers[0]);
    nH.getParam("cukeY", testCucumbers[1]);
    nH.getParam("cukeZ", testCucumbers[2]);

}

void worldNode::addCucumber() {

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
    cObj.primitive_poses[0].position.x = ex;
    cObj.primitive_poses[0].position.y = why;
    cObj.primitive_poses[0].position.z = zed;

    cObj.operation = moveit_msgs::CollisionObject::ADD;
    cObjPub.publish(cObj);
 
    planningSceneInterface->applyCollisionObject(cObj);

    std_msgs::Float32 desiredWorldPositionMsg;
    desiredWorldPositionMsg.data = ex;
    positionControlPub.publish(desiredWorldPositionMsg);
    ros::Duration(1.0).sleep();
    
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

// Prints the current attached objects on the end effector TODO put these in
// utility class?
void worldNode::printAttachedObjects() {

    // planningScene.robot_state.attached_collision_objects.clear();
    std::map<std::string, moveit_msgs::AttachedCollisionObject> aObjs;
    aObjs = planningSceneInterface->getAttachedObjects();
    if (aObjs.size() == 0) {
        ROS_INFO("There are no attached objects.");
    } else {
        std::map<std::string, moveit_msgs::AttachedCollisionObject>::iterator it;
        for (it = aObjs.begin(); it!=aObjs.end(); ++it) {
            ROS_INFO("Attached object : %s", (it->first).c_str());
        }
    }
}

// Define preset cartesian poses
void worldNode::defineCartesianPose() {

    tf::Quaternion q;
    
    // Place pose
    placePose.header.frame_id = "m1n6s200_link_base";
    placePose.header.stamp = ros::Time::now();

    // EULER ZYZ (-pi/4, pi/2, pi/2)
    // Dummy numbers for position since position is defined by cucumber
    // location
    placePose.pose.position.x = 0.2;
    placePose.pose.position.y = 0.4;
    placePose.pose.position.z = 0.1;

    // q = EulerZYZtoQuaternion(0, M_PI, M_PI/2);
    q = EulerZYZtoQuaternion(0, -M_PI, M_PI/2);
    placePose.pose.orientation.x = q.x(); // TODO could make this one line
    placePose.pose.orientation.y = q.y();
    placePose.pose.orientation.z = q.z();
    placePose.pose.orientation.w = q.w();
}

void worldNode::gripperAction(bool open) {
    if (open) {
      gripperGroupInterface->setNamedTarget("Open");
    } else {
      gripperGroupInterface->setNamedTarget("Close");
    }
    gripperGroupInterface->move();
}

// Closes or open gripper 
void worldNode::defineGripperPosture(bool open, trajectory_msgs::JointTrajectory &posture) {
    
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

// Move to drop off
void worldNode::moveToGoal() {
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    // Set the target pose
    armGroupInterface->setPoseTarget(placePose);

    success = (armGroupInterface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("planning successful ? : %d ", success);

    armGroupInterface->move();

}

// Receives a new cucumber
void worldNode::objectCallback(const moveit_msgs::CollisionObject &objectMsg) {
    

    ex  = objectMsg.primitives[0].pose.x; 
    why = objectMsg.primitives[0].pose.y;; 
    zed = objectMsg.primitives[0].pose.z;; 

    ROS_INFO("Cuke has been received with height: %f and radius: %f",
        objectMsg.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT],
        objectMsg.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);
    ROS_INFO("Picking cucumber with coordinates : X : %f, Y : %f, Z : %f", ex, why, zed);

    addCucumber();
    pickCucumber(cObj);
    moveToGoal();
    removeCucumber();
    gripperAction(true);
}

// Removes a collision object from the world
void worldNode::removeCucumber() {

    // Remove the attached object
    moveit_msgs::AttachedCollisionObject detachObject;
    detachObject.object.id = "target_cylinder";
    detachObject.link_name = "m1n6s200_end_effector";
    detachObject.object.operation = detachObject.object.REMOVE;// TODO

    // Create a planning scene diff message, remove the cucumber
    // from the gripper, and the world
    moveit_msgs::PlanningScene ps;
    ps.robot_state.attached_collision_objects.clear();
    ps.robot_state.attached_collision_objects.push_back(detachObject);
    ps.robot_state.is_diff = true;
    ps.world.collision_objects.clear();
    ps.world.collision_objects.push_back(detachObject.object);
    ps.is_diff = true;
    planningSceneInterface->applyPlanningScene(ps);
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
    q = EulerZYZtoQuaternion(M_PI/2, -M_PI/2, -M_PI/2);
    tf::quaternionTFToMsg(q, grasps[0].grasp_pose.pose.orientation);
    grasps[0].grasp_pose.pose.position.x = ex;
    grasps[0].grasp_pose.pose.position.y = why;
    grasps[0].grasp_pose.pose.position.z = zed;

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
    defineGripperPosture( true,  grasps[0].pre_grasp_posture);
    defineGripperPosture( false, grasps[0].grasp_posture);
    
    armGroupInterface->pick("target_cylinder", grasps);
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
