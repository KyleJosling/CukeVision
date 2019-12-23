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

        void objectCallback();

    private:
        
        // Node Handle
        ros::NodeHandle nH;

        const std::string nodeName = "worldNode";

        // Collision functions
        void addCucumber();
        void removeCucumber();


};
