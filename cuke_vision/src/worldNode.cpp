// ----------------------------------------------------------
// NAME: World Node
// DESCRIPTION: Node that handles adding and removing
// collision objects (cucumbers) from world map
// ----------------------------------------------------------
#include "cuke_vision/worldNode.hpp"

// Constructor
worldNode::worldNode() {
    
    cukeSub = nH.subscribe(cukeTopic, 100, &worldNode::objectCallback, this);
    cObjPub = nH.advertise<moveit_msgs::CollisionObject>(cObjTopic, 10);
    aObjPub = nH.advertise<moveit_msgs::AttachedCollisionObject>(aObjTopic, 10);

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

}
