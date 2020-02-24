// ----------------------------------------------------------
// NAME: FSM Node
// DESCRIPTION: This node controls the whole system through
// a sequence of states.
// ----------------------------------------------------------
#include "ControlFSM.hpp"
#include "ConcreteControlStates.hpp"

ControlFSM::ControlFSM() {
    
    // Initially in idle state
    currentState = &Idle::getInstance();


    // Subscriber to cucumber objects
    cukeSub = nH.subscribe(cucumberUnsortedTopic, 100, &ControlFSM::cucumberUnsortedCallback, this);

    // Stepper control publisher
    positionControlPub = nH.advertise<std_msgs::Float32>(positionControlTopic, 10);
    
    // Sorted cucumber publisher
    aObjPub = nH.advertise<moveit_msgs::CollisionObject>(cucumberSortedTopic, 10);

    // Immediately kick off the FSM
    toggle();
}

void ControlFSM::setState (ControlState &newState) {
    currentState->exit(this);
    currentState = &newState;
    currentState->enter(this);
}

void ControlFSM::toggle() {
    currentState->toggle(this);
}

void ControlFSM::cucumberUnsortedCallback(const moveit_msgs::CollisionObject &cucumberMsg) {
    cucumbers.push_back(cucumberMsg);
}

void ControlFSM::scanPlant() {


}

void ControlFSM::sortCucumbers() {
    
    std::sort(cucumbers.begin(), cucumbers.end(), sorter());

}
