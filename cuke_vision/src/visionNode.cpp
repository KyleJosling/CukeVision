// ----------------------------------------------------------
// NAME: Vision Node
// DESCRIPTION:
// 1. Receives depth and color images from Realsense camera
// 2. Detects and tracks cucumbers
// 3. Forms 3D objects from detected cucumbers using pixel
// deprojection
// ----------------------------------------------------------
#include "cuke_vision/visionNode.hpp"

// Initializer list
visionNode::visionNode():
    it(nH),
    colorImageSub(it, colorImageTopic, 1),
    depthImageSub(it, depthImageTopic, 1),
    sync(SyncPolicy(10), colorImageSub, depthImageSub) {
    
    // Initialize cucumber publisher
    cucumberPub = nH.advertise<moveit_msgs::CollisionObject>(cucumberTopic, 10);

    // Get intrinsics + extrinsics of camera
    cameraSetup();

    // Bind subscriber callbacks using boost
    sync.registerCallback(boost::bind( &visionNode::imageCallback, this, _1, _2));
}

// Destructor
visionNode::~visionNode() {

    // Destroy tracker objects
    for (int i = 0; i < cukeTrackers.size(); i++) {
        (cukeTrackers[i]).release();
    }
}

// Get intrinsic/extrinsic camera parameters
void visionNode::cameraSetup() {

    // Load the camera parameters
    sensor_msgs::CameraInfo cameraInfo;
    camInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10));

    if (camInfoPtr != NULL) {
        cameraInfo = *camInfoPtr;
    } else {
        ROS_ERROR("No camera parameters received. Shutting down vision node.");
        throw;
    }
}

// Image callback for depth and stereo images
void visionNode::imageCallback(const sensor_msgs::ImageConstPtr &colorImageMsg, const sensor_msgs::ImageConstPtr &depthImageMsg) {
    
    // TESTING roi
    cv::Rect roi;
    roi.x = 220; 
    roi.y = 0; 
    roi.width = 200; 
    roi.height = 480; 

    try {

        ROS_INFO("Frame received");

        int endian =  depthImageMsg->is_bigendian;
        std::cout << endian << std::endl;

        // Copy the frame
        colorFrame = cv_bridge::toCvCopy(colorImageMsg, "bgr8")->image;
        depthFrame = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        
        // Update tracker objects
        for (int i = 0; i < cukeTrackers.size(); i++) {

            (cukeTrackers[i])->update(colorFrame, cukeRegions[i]); 

            #ifdef GUI
            cv::rectangle(colorFrame, (cukeRegions[i]).tl(), (cukeRegions[i]).br(), cv::Scalar(0,0,255));
            #endif

            // If tracker has gone out of frame, release it
            if ((cukeRegions[i].x) <= 0 || (cukeRegions[i].x) >= IMAGE_WIDTH) {
                (cukeTrackers[i]).release();
                cukeTrackers.erase(cukeTrackers.begin()+i);
                cukeRegions.erase(cukeRegions.begin()+i);
            }
        }
        
        // Detect cucumbers
        boxes.clear();
        detector.detectCukes(colorFrame, boxes);

        for (int i = 0; i < boxes.size(); i++) {
            
            bool tracked = checkIfTracked(boxes[i]);
            
            // If new cucumber, create 3D object
            if (!tracked) {
                ROS_INFO("A new cucumber has appeared. Creating new tracking object.");
                draw3DBounding(boxes[i]);
                cukeTrackers.push_back(cv::TrackerKCF::create());
                (cukeTrackers.back())->init(colorFrame, boxes[i]);
                cukeRegions.push_back(boxes[i]);
            }
            
        }

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Image encoding error %s", e.what());
    }

}

// Checks if a cucumber is being tracked
bool visionNode::checkIfTracked(cv::Rect bounding) {

    // Cycle through the tracker objects
    for (int i = 0; i < cukeRegions.size(); i++) {
        int curXMin = (cukeRegions[i]).x; 
        int curXMax = (cukeRegions[i]).x + (cukeRegions[i]).width; 
        int curYMin = (cukeRegions[i]).y; 
        int curYMax = (cukeRegions[i]).y + (cukeRegions[i]).height;
        int centreX = bounding.x + bounding.width/2;
        int centreY = bounding.y + bounding.height/2;

        // Compare centre of detected to tracked ROI
        if (centreX > curXMin && centreX < curXMax && centreY > curYMin && centreY < curYMax)
            return true;
    }
    return false;
}

// Draw 3D bounding box
void visionNode::draw3DBounding(cv::Rect bounding) {

    // Get depth of front of cucumber
    float pixel_x    = bounding.x;
    float pixel_y    = bounding.y;
    float box_width  = bounding.width;
    float box_height = bounding.height;


    unsigned short frontDepth = depthFrame.at<unsigned short>( static_cast<int>(pixel_y + box_height/2), static_cast<int>(pixel_x + box_width/2));

    ROS_INFO("Checking depth at x : %d, y : %d , front depth : %d", static_cast<int>(pixel_x + box_width/2), static_cast<int>(pixel_y + box_height/2), frontDepth);

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
    
    ROS_INFO("Cucumber detected at X : %f, Y : %f, Z : %f and front depth : %u", pM.x, pM.y, pM.z, frontDepth);

    cObj.id = "cucumber";
    cObj.header.frame_id = "camera_link";

    // Define primitive and add its dimensions
    cObj.primitives.resize(1);
    cObj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    cObj.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    cObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.3; 
    cObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.02; 

    // Define the pose of the object
    cObj.primitive_poses.resize(1);
    cObj.primitive_poses[0].position.x = pM.z;
    cObj.primitive_poses[0].position.y = -pM.x;
    cObj.primitive_poses[0].position.z = -pM.y;

    cObj.operation = moveit_msgs::CollisionObject::ADD;
    cucumberPub.publish(cObj);

}

// Gets 3D coordinates of point
void visionNode::compute3DPoint(const float pixel_x, const float pixel_y, float depth, float (&point)[3]) {
    depth = depth*0.001f;
    float x = (pixel_x - camInfoPtr->K.at(2)) / camInfoPtr->K.at(0);
    float y = (pixel_y - camInfoPtr->K.at(5)) / camInfoPtr->K.at(4);

    point[0] = depth*x;
    point[1] = depth*y;
    point[2] = depth;
}

int main(int argc, char** argv) {

    std::string nodeName = "visionNode";
    ros::init(argc, argv, nodeName);

    visionNode sC;

    ros::spin();

    return 0;
}
