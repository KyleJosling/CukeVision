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
    
    // Initialize marker publisher
    markerPub = nH.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Get intrinsics + extrinsics of camera
    cameraSetup();

    // Bind subscriber callbacks using boost
    sync.registerCallback(boost::bind( &stereoCamNode::imageCallback, this, _1, _2));
}

// Get intrinsic/extrinsic camera parameters
void stereoCamNode::cameraSetup() {

    // Load the camera parameters
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
        colorFrame = cv_bridge::toCvCopy(colorImageMsg, "bgr8")->image;
        depthFrame = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

        // Detect cucumbers
        boxes.clear();
        points.points.clear();
        detector.detectCukes(colorFrame, boxes);
        // TODO should be boxes.size();
        for (int i = 0; i < 1; i++) {
            draw3DBounding(boxes[i]);
        }

        // Publish the message
        sendMarkers();

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Image encoding error %s", e.what());
    }

}

// Draw 3D bounding box
void stereoCamNode::draw3DBounding(cv::Rect bounding) {

    // std::cout << "Pixel at " << boxes[0].x + (boxes[0].width)/2 << " , " << boxes[0].y + (boxes[0].height)/2 << std::endl;
    // Get depth of front of cucumber
    float pixel_x    = bounding.x;
    float pixel_y    = bounding.y;
    float box_width  = bounding.width;
    float box_height = bounding.height;
    std::cout << "x, y " << static_cast<int>(pixel_x + box_width/2) << " " << static_cast<int>(pixel_y + box_height/2) << std::endl;

    // TODO should this be float
    unsigned short frontDepth = depthFrame.at<unsigned short>(static_cast<int>(pixel_x + box_width/2), static_cast<int>(pixel_y + box_height/2));
    std::cout << "front depth is : " << frontDepth << std::endl;

    float point[3];
    geometry_msgs::Point p;

    // Top left
    compute3DPoint(pixel_x, pixel_y, frontDepth, point);
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.points.push_back(p);

    // Top right
    compute3DPoint(pixel_x + box_width, pixel_y, frontDepth, point);
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.points.push_back(p);

    // Bottom left
    compute3DPoint(pixel_x, pixel_y + box_height, frontDepth, point);
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.points.push_back(p);

    // Bottom right
    compute3DPoint(pixel_x + box_width, pixel_y + box_height, frontDepth, point);
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.points.push_back(p);
}

// Gets 3D coordinates of point
void stereoCamNode::compute3DPoint(const float pixel_x, const float pixel_y, float depth, float (&point)[3]) {

    // float depth = depthFrame.at<unsigned short>(static_cast<int>(pixel_x), static_cast<int>(pixel_y));
    depth = depth*0.001f;
    float x = (pixel_x - camInfoPtr->K.at(2)) / camInfoPtr->K.at(0);
    float y = (pixel_y - camInfoPtr->K.at(5)) / camInfoPtr->K.at(4);

    point[0] = depth*x;
    point[1] = depth*y;
    point[2] = depth;
    std::cout << point[0] << std::endl;
    std::cout << point[1] << std::endl;
    std::cout << point[2] << std::endl;

}

// Sends visualization marker message
void stereoCamNode::sendMarkers() {
    
    // TODO could probably put all this repeated stuff in a function that's called once
    points.header.frame_id = "/camera_depth_optical_frame"; // TODO make this a constant
    points.header.stamp = ros::Time::now();
    points.ns = "point";
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.02;
    points.scale.y = 0.02;

    points.color.g = 1.0f;
    points.color.a = 1.0;
    
    markerPub.publish(points);
}

int main(int argc, char** argv) {

    std::string nodeName = "stereoCamNode";
    ros::init(argc, argv, nodeName);

    stereoCamNode sC;

    ros::spin();

    return 0;
}
