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

        // X Position
        double xPos;

        void transformLoop();

};
