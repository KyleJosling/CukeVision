// ----------------------------------------------------------
// NAME: Stepper Control Node
// DESCRIPTION: Node that controls the stepper motor for the
// linear drive system, and broadcasts a frame transform 
// accordingly
// ----------------------------------------------------------
#include "cuke_vision/stepperControlNode.hpp"

// Constructor
stepperControlNode::stepperControlNode() {

    // Initialize subscriber and publisher
    stepperPub = nH.advertise<std_msgs::Int32>(stepperControlTopic, 10);
    positionControlSub = nH.subscribe(positionControlTopic, 100, &stepperControlNode::messageCallback, this);
    
    // Set default x position
    xPos = 0.0;

    // Start broadcasting
    transformLoop();
}

stepperControlNode::~stepperControlNode() {

}

void stepperControlNode::transformLoop() {
    
    // tf::TransformBroadcaster broadcaster;
    // tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    ros::Rate rate(100);
    
    while (nH.ok()) {
        // Test stuff :
        // xPos+=0.0001;
        // transform.setOrigin(tf::Vector3(0,-xPos,0));
        // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "world"));

        // real stuff
        transform.setOrigin(tf::Vector3(-xPos,0,0));
        transform.setRotation(q);
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world")); // TODO add robot type
        ros::spinOnce();
        rate.sleep();
    }
}

void stepperControlNode::messageCallback(const std_msgs::Float32 &positionMsg) {
    xPos = positionMsg.data;
    xPos = (xPos - 0.2 < 0) ? xPos = 0 : xPos = xPos - 0.2; // Robot needs an offset to plan a path

    // Calculate stepper position
    int steps = -1 *((xPos*1000)/(0.067826)); // TODO reverse?

    ROS_INFO("STEPPER CONTROL : Position received with value %f", xPos);
    ROS_INFO("STEPPER CONTROL : Stepper position %d", steps);

    std_msgs::Int32 msg;
    msg.data = steps;
    stepperPub.publish(msg);
}

int main (int argc, char** argv ) {
    
    std::string nodeName = "stepperControlNode";
    ros::init(argc, argv, nodeName);
    
    stepperControlNode w;

    // ros::spin();

    return 0;
}
