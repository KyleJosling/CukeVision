// ----------------------------------------------------------
// NAME: World Node
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
#include "cuke_vision/worldNode.hpp"

const double FINGER_MAX = 6400;

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
    // planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();
    planningScene.reset(new planning_scene::PlanningScene(robotModel)); 
    planningSceneMonitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // Initialize the move group interface and planning scene interface
    armGroupInterface = new moveit::planning_interface::MoveGroupInterface(armPlanningGroup);
    gripperGroupInterface = new moveit::planning_interface::MoveGroupInterface(gripperPlanningGroup);

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
    // moveToGoal();
    // addTable();
    // pickCucumber();
    gripperAction(6400);
}

// World destructor
worldNode::~worldNode() {

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
    graspPose.header.frame_id = "root";
    graspPose.header.stamp = ros::Time::now();

    // EULER ZYZ (-pi/4, pi/2, pi/2)
    graspPose.pose.position.x = 0.0;
    graspPose.pose.position.y = 0.6;
    graspPose.pose.position.z = 0.3;
}

// Closes or open gripper 
bool worldNode::gripperAction(double fingerTurn) {
    
    
    if (robotConnected == false) {
        
        if (fingerTurn>0.5*FINGER_MAX) {
            gripperGroupInterface->setNamedTarget("Close");
        } else {
            gripperGroupInterface->setNamedTarget("Open");
        }
        gripperGroupInterface->move();
        return true;
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = fingerTurn;
    goal.fingers.finger2 = fingerTurn;
    goal.fingers.finger3 = fingerTurn;
    fingerClient->sendGoal(goal);
    
    if (fingerClient->waitForResult(ros::Duration(5.0))) {
        fingerClient->getResult();
        return true;
    } else {
        fingerClient->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

// TODO test function
void worldNode::moveToGoal() {
    
    geometry_msgs::PoseStamped randomPose = armGroupInterface->getRandomPose();    

    // Set the target pose
    armGroupInterface->setPoseTarget(randomPose);

    // TODO make member variable?
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (armGroupInterface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("planning successful ? : %d ", success);

    armGroupInterface->move();
}

// Receives a new cucumber
void worldNode::objectCallback(const moveit_msgs::CollisionObject &objectMsg) {
    
    ROS_INFO("Cuke has been received with height: %f and radius: %f",
        objectMsg.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT],
        objectMsg.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);

    cObjPub.publish(objectMsg);

    // planningSceneInterface->applyCollisionObject(objectMsg);
}

// Removes a collision object from the world
void worldNode::removeCucumber() {

}

// Pick cucumber
void worldNode::pickCucumber() {
    
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);


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
