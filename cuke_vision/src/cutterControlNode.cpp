// ----------------------------------------------------------
// NAME: Cutter Control Node
// DESCRIPTION: Forwards cutter commands to Arduino
// ----------------------------------------------------------
#include "cuke_vision/cutterControlNode.hpp"

// Constructor
cutterControlNode::cutterControlNode() {

    // Initialize subscriber and publisher
    cutterPub = nH.advertise<std_msgs::Bool>(cutterControlSerialTopic, 10);
    cutterControlSub = nH.subscribe(cutterControlTopic, 100, &cutterControlNode::messageCallback, this);

}

cutterControlNode::~cutterControlNode() {

}

void cutterControlNode::messageCallback(const std_msgs::Bool &positionMsg) {

}

int main (int argc, char** argv ) {

    std::string nodeName = "cutterControlNode";
    ros::init(argc, argv, nodeName);

    cutterControlNode w;

    return 0;
}
