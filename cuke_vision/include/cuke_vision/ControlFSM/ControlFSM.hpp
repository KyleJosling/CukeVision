// ----------------------------------------------------------
// NAME: FSM Node Header
// DESCRIPTION: This node controls the whole system through
// a sequence of states.
// ----------------------------------------------------------


#pragma once
#include "ControlState.hpp"

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

// Forward declaration to resolve circular dependency/include
class ControlState;

class ControlFSM {

    public:

        // Initialization list
        ControlFSM();

        inline controlState* getCurrentState() const { return currentState;}
        void toggle();
        void setState(controlState &newState);

        // Plant scanning
        void scanPlant();

        // Cucumber sorting
        void sortCucumbers();


    private:

        ControlState *currentState;

        // Node Handle
        ros::NodeHandle nH;

        // Node Name
        const std::string nodeName = "ControlFSM";

        // Topics
        const std::string cucumberUnsortedTopic = "cuke3DUnsorted";
        const std::string cucumberSortedTopic = "cuke3DSorted";
        const std::string positionControlTopic = "positionControl";

        // Publishers
        ros::Publisher positionControlPub;
        ros::Publisher sortedCucumberPub;

        // Subscriber
        ros::Subscriber unsortedCucumberSub;

        // Cucumber vector
        std::vector<moveit_msgs::CollisionObject> cucumbers;

};

// Sorter for cucumbers
struct sorter {
    
    inline bool operator() (const moveit_msgs::CollisionObject &obj1, const moveit_msgs::CollisionObject &obj2) {
        return (obj1.primitive_poses.position.x < obj2.primitive_poses.position.x);
    }

};
