// ----------------------------------------------------------
// NAME: FSM Node
// DESCRIPTION: This node controls the whole system through
// a sequence of states.
// ----------------------------------------------------------
#pragma once
#include "ControlState.hpp"

#include <ros/ros.h>
#include <iostream>

// Forward declaration to resolve circular dependency/include
class ControlState;

class controlNode {

    public:

        // Initialization list
        controlFSM();

        inline controlState* getCurrentState() const { return currentState;}
        void toggle();
        void setState(controlState &newState);

    private:

        ros::NodeHandle nH;
        
       //  // States
       //  enum controlStates {
       //      
       //      ST_START_UP,
       //      ST_SCAN_CUCUMBERS,
       //      ST_SORT_CUCUMBERS,
       //      ST_SAVE_OCCUPANCY_MAP,
       //      ST_MOVEIT_START_UP,
       //      ST_SHUT_DOWN
       //  };
            
        ControlState *currentState;

};
