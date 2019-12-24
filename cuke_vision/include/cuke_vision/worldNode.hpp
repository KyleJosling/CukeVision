// ----------------------------------------------------------
// NAME: Stereo Camera Node
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


class worldNode {

    public:
        worldNode();
        ~worldNode();

        void objectCallback(const shape_msgs::SolidPrimitive &objectMsg);

    private:
        
        // Node Handle
        ros::NodeHandle nH;

        const std::string nodeName = "worldNode";
        const std::string cukeTopic = "cuke3D";
        const std::string cObjTopic = "cObj";
        const std::string aObjTopic = "aObj";

        // Publishers
        ros::Publisher cObjPub;
        ros::Publisher aObjPub;

        // Subscribers
        ros::Subscriber cukeSub;

        // Collision objects
        moveit_msgs::CollisionObject cObj;
        moveit_msgs::AttachedCollisionObject aObj;
        
        // Collision functions
        void addCucumber();
        void removeCucumber();


};

