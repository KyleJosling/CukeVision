// ----------------------------------------------------------
// NAME: World Node Header
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
// TODO organize

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

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
        const std::string positionControlTopic = "positionControl";

        // Publishers
        ros::Publisher cObjPub;
        ros::Publisher aObjPub;
        ros::Publisher positionControlPub;

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
        boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> armGroupInterface;
        boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> gripperGroupInterface;
        robot_model::RobotModelPtr robotModel;

        // Planning scene interface
        boost::scoped_ptr<moveit::planning_interface::PlanningSceneInterface> planningSceneInterface;

        // Model of robot
        moveit::core::RobotStatePtr currentRobotState;

        // Various poses
        geometry_msgs::Pose homePose;
        geometry_msgs::PoseStamped graspPose;
        geometry_msgs::PoseStamped placePose;

        // Collision objects
        moveit_msgs::CollisionObject cObj; // Current cucumber to pick
        moveit_msgs::AttachedCollisionObject aObj; // Current attached cucumber
        
        // Cucumbers
        std::stack<moveit_msgs::CollisionObject> cucumbers;
        
        // Test cucumbers
        std::vector<std::vector<double>> testCucumbers;
        double ex, why, zed;

        // Transform listener
        tf::TransformListener listener;

        void initializeTransformListener();
        void moveToGoal();
        void defineGripperPosture(bool open, trajectory_msgs::JointTrajectory &posture);
        void addPermanentObjects();
        void gripperAction(bool open);

        // Utilities
        void printRobotPose();
        void printAttachedObjects();
        void loadCucumbersFromFile();

        // Collision functions
        void addCucumber();
        void pickCucumber(const moveit_msgs::CollisionObject &objectMsg);
        void removeCucumber();
        void defineCartesianPose();

};
