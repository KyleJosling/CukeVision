// ----------------------------------------------------------
// NAME: FSM Node State Definitions
// DESCRIPTION: This file defines the processes that run 
// in each control state (enter, toggle, and exit), and also
// defines state transitions
// ----------------------------------------------------------

#include "ConcreteControlStates.hpp"


void Idle::toggle(ControlFSM *control) {

    control->setState(SortCucumbers::StartUp());
}

ControlState &Idle::getInstance() {
    
    static Idle singleton;
    return singleton;
}

// ------------ START UP STATE ------------ //
//
void StartUp::enter(ControlFSM *control) {

    // Launch the Intel D435 camera node
    ROS_INFO("Launching realsense node.");
    system("roslaunch rs_camera rs_camera.launch enable_pointcloud:="true" &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();

    // Launch the python rosserial node.
    ROS_INFO("Launching the python rosserial node to communicate with the Arduino.");
    system("rosrun rosserial_python serial_node.py /dev/ttyACM0");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();

    // Launch the stepper control node.
    ROS_INFO("Launching the stepper control node.");
    system("rosrun cuke_vision stepperControlNode &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();

    // Launch the cutter control node.
    ROS_INFO("Launching the cutter control node.");
    system("rosrun cuke_vision cutterControlNode &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();

    // Launch a static frame transform between the camera and the robot's base.
    ROS_INFO("Launching a static frame transform between camera and robot's base.");
    system("rosrun tf static_transform_publisher 0.5 0.35 0.10 3.14 0 0 /root /camera_link 100 &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();

    // Start the Octomap server.
    ROS_INFO("Starting octomap server node.");
    system("roslaunch octomap_server octomap_custom.launch &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();


    // Move to the next state
    control->toggle();
}

void StartUp::toggle(ControlFSM *control) {

    control->setState(ScanCucumbers::getInstance());
}

ControlState &StartUp::getInstance() {
    
    static StartUp singleton;
    return singleton;
}


// ------------ CUCUMBER SCANNING STATE ------------ //

void ScanCucumbers::enter(ControlFSM *control) {
    


    // Perform 5 scans of the cucumber
    for (int i = 0; i < 4; i++) {
        control->scanPlant();
    }

    // Start the image processing node for the last scan
    ROS_INFO("Starting vision node.");
    system("rosrun cuke_vision visionNode &");
    control->scanPlant();

    // Go to next state
    control->toggle();
}

void ScanCucumbers::toggle(ControlFSM *control) {

    control->setState(SortCucumbers::getInstance());
}


void ScanCucumbers::exit(ControlFSM *control) {

    // Kill the vision node on Scan Cucumber state exit
    system("roskill cuke_vision visionNode &");
    system("roskill rs_camera &");
}

ControlState &ScanCucumbers::getInstance() {
    
    static ScanCucumbers singleton;
    return singleton;
}


// ------------ CUCUMBER SORTING STATE ------------ //

void SortCucumbers::enter(ControlFSM *control) {
    
    control->sortCucumbers();
    control->toggle();
}

void SortCucumbers::toggle(ControlFSM *control) {

    control->setState(SaveOccupancyMap::getInstance());
}

ControlState &SortCucumbers::getInstance() {
    
    static SortCucumbers singleton;
    return singleton;
}

// ------------ SAVE OCCUPANCY MAP STATE ------------ //

void SaveOccupancyMap::enter(ControlFSM *control) {

    system("roskill octomap_server &");
    control->toggle();
}

void SaveOccupancyMap::toggle(ControlFSM *control) {

    control->setState(MoveItStartUp::getInstance());
}

ControlState &SaveOccupancyMap::getInstance() {
    
    static SaveOccupancyMap singleton;
    return singleton;
}

// ------------ MOVEIT START UP STATE ------------ //

void MoveItStartUp::enter(ControlFSM *control) {

    ROS_INFO("Starting MoveIt!");
    system("roslaunch m1n6s200_moveit_config m1n6s200_demo.launch &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();

    ROS_INFO("Starting World Node");
    system("rosrun cuke_vision worldNode &");

    // Wait for node to launch ..
    ros::Duration(5.0).sleep();
}

void MoveItStartUp::toggle(ControlFSM *control) {

    control->setState(MoveItStartUp::getInstance());
}

ControlState &MoveItStartUp::getInstance() {
    
    static MoveItStartUp singleton;
    return singleton;
}

// ------------ SHUTDOWN STATE ------------ //

void ShutDown::enter(ControlFSM *control) { 

    system("roskill m1n6s200_moveit_config &");
    system("roskill cuke_vision worldNode &");
    system("roskill cuke_vision stepperControlNode &");
    system("roskill cuke_vision cutterNode &");
    system("roskill rosserial_python serial_node.py &");
    system("roskill tf static_transform_publisher &");
    // Stay in shutdown state forever
}

void ShutDown::toggle(ControlFSM *control) {

    control->setState(ShutDown::getInstance());
}

ControlState &ShutDown::getInstance() {
    
    static ShutDown singleton;
    return singleton;
}
