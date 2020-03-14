// ----------------------------------------------------------
// NAME: Vision Node
// DESCRIPTION:
// 1. Receives depth and color images from Realsense camera
// 2. Detects and tracks cucumbers
// 3. Forms 3D objects from detected cucumbers using pixel
// deprojection
// ----------------------------------------------------------

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking/tracker.hpp>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"


#include "cuke_vision/cukeDetector.hpp"

#include <iostream>
#include <stdio.h>

class visionNode {

    public:
        
        // Constructor
        visionNode();

        // Destructor
        ~visionNode();

    private:
        
        // Node variables
        ros::NodeHandle nH;
        image_transport::ImageTransport it;
        const std::string nodeName = "visionNode";
        
        // Topics
        const std::string cameraInfoTopic = "/camera/aligned_depth_to_color/camera_info";
        const std::string colorImageTopic = "/camera/color/image_raw";
        const std::string depthImageTopic = "/camera/aligned_depth_to_color/image_raw";
        const std::string detectedImageTopic = "detectedCucumbers";
        const std::string cucumberTopic = "cuke3D";

        // Constants
        const int IMAGE_WIDTH  = 640;
        const int IMAGE_HEIGHT = 480;

        // Image transport subscriber
        image_transport::SubscriberFilter colorImageSub;
        image_transport::SubscriberFilter depthImageSub;

        // Image transport publisher
        image_transport::Publisher detectedImagePub;

        // Cucumber publisher
        ros::Publisher cucumberPub;

        // Sync policy for synchronization
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

        // Define synchronizer for callbacks
        message_filters::Synchronizer<SyncPolicy> sync;
        
        // Cucumber detector object
        cukeDetector detector;

        // Tracker objects for cucumbers
        std::vector<cv::Ptr<cv::TrackerKCF>> cukeTrackers;
        std::vector<cv::Rect2d> cukeRegions;

        // Current frame and bounding boxes around cukes
        cv::Mat colorFrame;
        cv::Mat depthFrame;
        sensor_msgs::ImagePtr detectedMsg;
        std::vector<cv::Rect> boxes;

        // Collision objects (cucumbers we have detected)
        moveit_msgs::CollisionObject cObj;
        std::vector<moveit_msgs::CollisionObject> cObjs;
         
        // Camera intrinsics/extrinsics
        boost::shared_ptr<const sensor_msgs::CameraInfo> camInfoPtr;
        float K[9];

        void cameraSetup();
        void imageCallback(const sensor_msgs::ImageConstPtr &colorImageMsg, const sensor_msgs::ImageConstPtr &depthImageMsg);
        bool checkIfTracked(cv::Rect bounding);
        void draw3DBounding(cv::Rect bounding);
        void compute3DPoint(const float pixel_x, const float pixel_y, float depth, float (&point)[3]);
        void sendObjects();
};
