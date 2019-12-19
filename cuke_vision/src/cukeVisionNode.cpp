#include <ros/ros.h>
#include <iostream>

#include "cuke_vision/detectObject.hpp"
#include "cuke_vision/boundingBoxMsg.h"

class cukeVisionNode {

    public:

        // Initialization list
        cukeVisionNode() { 
            
            // Initialize publisher 
            // boundingBoxPub = nH.advertise<cuke_vision::boundingBoxMsg>(boxTopic, 1000);
        }

    private:
        
        const std::string boxTopic = "/2d_bounding_boxes";

        ros::NodeHandle nH;

        // Publisher
        ros::Publisher boundingBoxPub;
};

int main(int argc, char** argv) {

    std::string nodeName = "cukeVisionNode";
    ros::init(argc, argv, nodeName);

    ros::spin();

    return 0;
}


