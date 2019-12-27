// ----------------------------------------------------------
// NAME: World Node
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
#include "cuke_vision/worldNode.hpp"

// Constructor
worldNode::worldNode() {
    
    // Initialize subscribers and publishers
    cukeSub = nH.subscribe(cukeTopic, 100, &worldNode::objectCallback, this);
    cObjPub = nH.advertise<moveit_msgs::CollisionObject>(cObjTopic, 10);
    aObjPub = nH.advertise<moveit_msgs::AttachedCollisionObject>(aObjTopic, 10);

    // Get robot type, check if robot is connected
    nH.param<std::string>("/robot_type", robotType);
    nH.param<bool>("/robot_connected", robotConnected);

    // Initialize the move group interface and planning scene interface
    armGroupInterface = new moveit::planning_interface::MoveGroupInterface(armPlanningGroup);
    // planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface(gripperPlanningGroup);
    
    // Initialize joint model group
    jointModelGroup = (armGroupInterface->getCurrentState())->getJointModelGroup(armPlanningGroup);


    // Add visual tools
    // TODO clean up
    #ifdef GUI
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm_link0");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", (armGroupInterface->getPlanningFrame()).c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", (armGroupInterface->getEndEffectorLink()).c_str());
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    #endif
    
}

worldNode::~worldNode() {

}

// Receives a new cucumber
void worldNode::objectCallback(const shape_msgs::SolidPrimitive &objectMsg) {

}

// Adds a collision object to the world
void worldNode::addCucumber() {
    
    cObj.operation = moveit_msgs::CollisionObject::ADD;
    cObjPub.publish(cObj);

}

// Removes a collision object from the world
void worldNode::removeCucumber() {

}

int main (int argc, char** argv ) {
    
    std::string nodeName = "worldNode";
    ros::init(argc, argv, nodeName);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    worldNode w;

    ros::spin();
    return 0;
}
