// ----------------------------------------------------------
// NAME: Stereo Camera Node
// DESCRIPTION:
// 1. Receives depth and color images from Realsense camera
// 2. Gets 3D points from pixel coordinates
// ----------------------------------------------------------
#include "cuke_vision/stereoCamNode.hpp"

// Initializer list
stereoCamNode::stereoCamNode():
    it(nH),
    colorImageSub(it, colorImageTopic, 1),
    depthImageSub(it, depthImageTopic, 1),
    sync(SyncPolicy(10), colorImageSub, depthImageSub) {
    
    // Get intrinsics + extrinsics of camera
    cameraSetup();

    // Bind subscriber callbacks using boost
    sync.registerCallback(boost::bind( &stereoCamNode::imageCallback, this, _1, _2));
}

// Get intrinsic + extrinsic camera parameters
void stereoCamNode::cameraSetup() {

    // Load the camera parameters
    boost::shared_ptr<const sensor_msgs::CameraInfo> camInfoPtr;
    sensor_msgs::CameraInfo cameraInfo;
    camInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10));

    if (camInfoPtr != NULL) {
        cameraInfo = *camInfoPtr;
    } else {
        ROS_ERROR("No camera parameters received. Shutting down CukeVisionNode.");
        throw;
    }
}

// Image callback for depth and stereo images
void stereoCamNode::imageCallback(const sensor_msgs::ImageConstPtr &colorImageMsg, const sensor_msgs::ImageConstPtr &depthImageMsg) {

    try {

        ROS_INFO("Frame received");

        // Copy the frame
        frame = cv_bridge::toCvCopy(colorImageMsg, "bgr8")->image;
        boxes = detector.detectCukes(frame);

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Image encoding error %s", e.what());
    }

}

void stereoCamNode::compute3DPoint(float x, float y) {

}


int main(int argc, char** argv) {

    std::string nodeName = "stereoCamNode";
    ros::init(argc, argv, nodeName);

    stereoCamNode sC;

    ros::spin();

    return 0;
}
