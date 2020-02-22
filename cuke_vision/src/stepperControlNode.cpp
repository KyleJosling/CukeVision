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

    // Calculate relative stepper position
    int steps = ((desiredPos-oldPos)*1000)/(millimetresPerStep);
    double distToMaxSpeed = maxStepperSpeed*maxStepperSpeed/(2*stepperAcceleration); //steps
    double timeToMaxSpeed = 2*distToMaxSpeed/maxStepperSpeed;


    ROS_INFO("STEPPER CONTROL : Position received with value %f", desiredPos);
    ROS_INFO("STEPPER CONTROL : Steps to travel %d", steps);


    // Publish the steps and move the motor
    std_msgs::Int32 msg;
    msg.data = -steps;
    stepperPub.publish(msg);

    ros::Time startTime = ros::Time::now();
    ros::Time currentTime = startTime;
    double deltaTime = (currentTime - startTime).toSec();

    //accel until max speed, decel at desiredPos - distToMaxSpeed
    if (abs(steps) > distToMaxSpeed) {

        // Calculate the time to execute the steps
        trajectoryTime = (2*timeToMaxSpeed) + ((steps-(2*distToMaxSpeed))/maxStepperSpeed);
        double maxDecelTime = trajectoryTime - timeToMaxSpeed;

        while (deltaTime <= trajectoryTime) {

            currentTime = ros::Time::now();
            deltaTime = (currentTime - startTime).toSec();


            if (deltaTime < timeToMaxSpeed) {
                Pos = oldPos + stepperAcceleration*deltaTime*deltaTime/2 *(millimetresPerStep/1000);
                Pos1 = Pos;
            } else if (timeToMaxSpeed <= deltaTime <= maxDecelTime ) {
                Pos = Pos1 + ((deltaTime-timeToMaxSpeed)*maxStepperSpeed) *(millimetresPerStep/1000);
                Pos2 = Pos;
            } else {
                Pos = Pos2 + stepperAcceleration*pow((deltaTime-maxDecelTime),2)/2 *(millimetresPerStep/1000);
            }

            // Publish new position
            transform.setOrigin(tf::Vector3(-Pos,0,0));
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world"));
        }

    // If the max speed wont be reached: accel=decel, so accelerate until steps/2
    } else {

        // Calculate the time to execute the steps
        trajectoryTime = sqrt(4*steps/stepperAcceleration);

        while (deltaTime <= trajectoryTime) {

            currentTime = ros::Time::now();
            deltaTime = (currentTime - startTime).toSec();
            Pos = oldPos + stepperAcceleration*deltaTime*deltaTime/2 *(millimetresPerStep/1000);

            // Publish new position
            transform.setOrigin(tf::Vector3(-Pos,0,0));
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world"));

        }
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
