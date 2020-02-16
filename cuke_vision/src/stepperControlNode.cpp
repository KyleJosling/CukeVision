// ----------------------------------------------------------
// NAME: Stepper Control Node
// DESCRIPTION: Node that controls the stepper motor for the
// linear drive system, and broadcasts a frame transform 
// accordingly
// ----------------------------------------------------------
#include "cuke_vision/stepperControlNode.hpp"

// Constructor
stepperControlNode::stepperControlNode() {


}

stepperControlNode::~stepperControlNode() {

}

void stepperControlNode::transformLoop() {

    while (n.ok()) {
        
        transform.setOrigin(tf::Vector3(0,0,0));
        transform.setRotation(tf::Quaternion(0,0,0,0));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world"));
    }
}

void stepperControlNode::messageCallback(const std_msgs::Float32 &positionMsg) {

}

int main (int argc, char** argv ) {
    
    std::string nodeName = "stepperControlNode";
    ros::init(argc, argv, nodeName);
    
    stepperControlNode w;

    ros::spin();
    return 0;
}
