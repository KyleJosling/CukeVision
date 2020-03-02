// ----------------------------------------------------------
// NAME: Cutter Control Node Header
// DESCRIPTION: Header for node that forwards cutter commands
// to Arduino
// ----------------------------------------------------------

#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>


class cutterControlNode {

    public:
        cutterControlNode();
        ~cutterControlNode();
        void messageCallback(const std_msgs::Bool &positionMsg);

    private:

        // Node handle
        ros::NodeHandle nH;

        // Node name
        const std::string nodeName = "cutterControlNode";

        // Topics
        const std::string cutterControlTopic = "cutterControl";
        const std::string cutterControlSerialTopic = "cutterControlSerial";

        // Publisher and subscriber
        ros::Subscriber cutterControlSub;
        ros::Publisher  cutterPub;;

};
