// ----------------------------------------------------------
// NAME: World Node Header
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
// TODO organize

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>

#include <kinova_driver/kinova_ros_types.h>
#include <kinova_msgs/SetFingersPositionAction.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/solid_primitive_dims.h>


class worldNode {

    public:
        worldNode();
        ~worldNode();

        void objectCallback(const moveit_msgs::CollisionObject &objectMsg);

    private:
        
        // Node Handle
        ros::NodeHandle nH;
        
        // Node name
        const std::string nodeName = "worldNode";

        // Topics
        const std::string cukeTopic = "cuke3D";
        const std::string cObjTopic = "cObj";
        const std::string aObjTopic = "aObj";

        // Publishers
        ros::Publisher cObjPub;
        ros::Publisher aObjPub;

        // Subscribers
        ros::Subscriber cukeSub;

        // Planning group name (the set of robot joints)
        const std::string armPlanningGroup = "arm";
        const std::string gripperPlanningGroup = "gripper";

        // Other parameters
        std::string robotType;
        bool robotConnected;
        
        // Action client for moving fingers
        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* fingerClient;

        // Planning interfaces for arm and gripper TODO should these be pointers?
        moveit::planning_interface::MoveGroupInterface *armGroupInterface;
        moveit::planning_interface::MoveGroupInterface *gripperGroupInterface;
        robot_model::RobotModelPtr robotModel;

        // Planning scene interface
        moveit::planning_interface::PlanningSceneInterface *planningSceneInterface;


        planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor;
        planning_scene::PlanningScenePtr planningScene;

        // Model of robot
        moveit::core::RobotStatePtr currentRobotState;

        // Various poses
        geometry_msgs::Pose homePose;
        geometry_msgs::PoseStamped graspPose;
        geometry_msgs::PoseStamped placePose;

        // Collision objects
        moveit_msgs::CollisionObject cObj;
        moveit_msgs::AttachedCollisionObject aObj;
        
        // Cucumbers
        std::stack<moveit_msgs::CollisionObject> cucumbers;
        

        void moveToGoal();
        void gripperAction(bool open, trajectory_msgs::JointTrajectory &posture);

        // Utilities
        void printRobotPose();

        // Collision functions
        void pickCucumber(const moveit_msgs::CollisionObject &objectMsg);
        // void addTable();
        void removeCucumber();
        void defineCartesianPose();
        void addTestObject();

};

