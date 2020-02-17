// ----------------------------------------------------------
// NAME: Stepper Control Node Header
// DESCRIPTION: Header for node that controls the stepper 
// motor for the linear drive system, and broadcasts a frame 
// transform accordingly
// ----------------------------------------------------------

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <tf/transform_broadcaster.h>


class stepperControlNode {

    public:
        stepperControlNode();
        ~stepperControlNode();
        void messageCallback(const std_msgs::Float32 &positionMsg);

    private:
        
        // Node handle
        ros::NodeHandle nH;

        // Node name
        const std::string nodeName = "stepperControlNode";

        // Topics
        const std::string positionControlTopic = "positionControl";
        const std::string stepperControlTopic = "stepperControl";

        // Publisher and subscriber
        ros::Subscriber positionControlSub;
        ros::Publisher  stepperPub;

        // Transform
        tf::TransformBroadcaster broadcaster;
        tf::Transform transform;

        // mm/step, calculated using a 1.36" sprocket diameter
        // and 1600 microsteps/revolution
        const double millimetresPerStep = 0.067826;
        
        // Acceleration in steps/s^2
        const double stepperAcceleration = 10000;

        // Speed in steps/s
        const double stepperSpeed = 10000;

        // X Positions (metres)
        double Pos;
        double desiredPos;
        double oldPos;

        void transformLoop();

};
