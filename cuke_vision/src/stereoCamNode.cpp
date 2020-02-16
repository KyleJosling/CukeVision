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
    
    // Initialize cucumber publisher
    cucumberPub = nH.advertise<moveit_msgs::CollisionObject>(cucumberTopic, 10);

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
        
        detector.detectCukes(colorFrame, boxes);
        ROS_INFO("Meeperson");
        // TODO should be boxes.size();
        for (int i = 0; i < 1; i++) {
            if (boxes.size() > 0)
                draw3DBounding(boxes[i]);
        }

        // Publish the message
        // sendObjects();

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

    // Points on plane on the front of the cucumber
    float point[3];
    geometry_msgs::Point pM, pTL, pTR, pBL, pBR;

    // Centre point
    compute3DPoint(pixel_x + box_width/2, pixel_y + box_height/2, frontDepth, point);
    pM.x = point[0];
    pM.y = point[1];
    pM.z = point[2];

    // Top left
    compute3DPoint(pixel_x, pixel_y, frontDepth, point);
    pTL.x = point[0];
    pTL.y = point[1];
    pTL.z = point[2];

    // Top right
    compute3DPoint(pixel_x + box_width, pixel_y, frontDepth, point);
    pTR.x = point[0];
    pTR.y = point[1];
    pTR.z = point[2];

    // Bottom left
    compute3DPoint(pixel_x, pixel_y + box_height, frontDepth, point);
    pBL.x = point[0];
    pBL.y = point[1];
    pBL.z = point[2];

    // Bottom right
    compute3DPoint(pixel_x + box_width, pixel_y + box_height, frontDepth, point);
    pBR.x = point[0];
    pBR.y = point[1];
    pBR.z = point[2];
    
    // // TODO can seperate this into one function
    cObj.id = "cucumber";
    cObj.header.frame_id = "camera_link";

    // Define primitive and add its dimensions
    cObj.primitives.resize(1);
    cObj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    cObj.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    cObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.33; // TODO these are hardcoded bc of bug in last approach, FIX
    cObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.02;

    // Define the pose of the object
    cObj.primitive_poses.resize(1);
    cObj.primitive_poses[0].position.x = pM.z;
    cObj.primitive_poses[0].position.y = -pM.x;
    cObj.primitive_poses[0].position.z = -pM.y;

    std::cout << "X Y Z of cucumber is : " << pM.x << " " << pM.y << " " << pM.z << " " << std::endl;
    cObj.operation = moveit_msgs::CollisionObject::ADD;
    cucumberPub.publish(cObj);

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
    // std::cout << point[0] << std::endl;
    // std::cout << point[1] << std::endl;
    // std::cout << point[2] << std::endl;
}

int main(int argc, char** argv) {

    std::string nodeName = "stereoCamNode"; // TODO should this happen in object constructor ..
    ros::init(argc, argv, nodeName);

    stereoCamNode sC;

    ros::spin();

    return 0;
}
