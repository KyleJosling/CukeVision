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
    Pos = 0.0;

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
        // Pos+=0.0001;
        // transform.setOrigin(tf::Vector3(0,-Pos,0));
        // transform.setRotation(q);
        // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "world"));

        // real stuff
        transform.setOrigin(tf::Vector3(-Pos,0,0));
        transform.setRotation(q);
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world"));
        ros::spinOnce();
        rate.sleep();
    }
}

void stepperControlNode::messageCallback(const std_msgs::Float32 &positionMsg) {
    
    oldPos = Pos;
    desiredPos = positionMsg.data;
    desiredPos = (desiredPos - 0.2 < 0) ? desiredPos = 0 : desiredPos = desiredPos - 0.2; // Robot needs an offset to plan a path

    // Calculate stepper position
    int steps = ((desiredPos*1000)/(millimetresPerStep));

    // Calculate the time to execute the steps
    double trajectoryTime = steps/stepperSpeed;

    ROS_INFO("STEPPER CONTROL : Position received with value %f", desiredPos);
    ROS_INFO("STEPPER CONTROL : Stepper position %d", steps);
    ROS_INFO("STEPPER CONTROL : Stepper will execute this trajectory in %f seconds", trajectoryTime);
    
    // Publish the steps and move the motor
    std_msgs::Int32 msg;
    msg.data = -steps;
    stepperPub.publish(msg);

    // Transform publisher that will broadcast robot odometry
    ros::Time startTime = ros::Time::now();
    ros::Time currentTime = startTime;
    double deltaTime = (currentTime - startTime).toSec();

    while (deltaTime <= trajectoryTime) {
        
        currentTime = ros::Time::now();
        deltaTime = (currentTime - startTime).toSec();

        // Calculate how far the robot has moved since the timer started 
        // if (deltaTime < 1) {
        //     Pos = oldPos + (deltaTime * deltaTime * stepperAcceleration)*(millimetresPerStep/1000);
        // } else {
            Pos = oldPos + (deltaTime * stepperSpeed)*(millimetresPerStep/1000);
        // }
        
        // Publish new position
        transform.setOrigin(tf::Vector3(-Pos,0,0));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world"));
    }

    // Calculate error
    double error = Pos - desiredPos;
    ROS_INFO("STEPPER CONTROL : Real robot position : %f, Transform position : %f, Error : %f", Pos, desiredPos, error);
    Pos = desiredPos;

}

int main (int argc, char** argv ) {
    
    std::string nodeName = "stepperControlNode";
    ros::init(argc, argv, nodeName);
    
    stepperControlNode w;

    return 0;
}
