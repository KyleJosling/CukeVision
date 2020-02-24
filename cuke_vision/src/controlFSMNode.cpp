// ----------------------------------------------------------
// NAME: FSM Node
// DESCRIPTION: This node controls the whole system through
// a sequence of states.
// ----------------------------------------------------------
#include "ControlFSM.hpp"


int main(int argc, char** argv) {

    std::string nodeName = "controlFSM";
    ros::init(argc, argv, nodeName);

    ros::spin();

    return 0;
}


