// ----------------------------------------------------------
// NAME: Stereo Camera Node Header
// DESCRIPTION:
// 1. Receives depth and color images from Realsense camera
// 2. Gets 3D points from pixel coordinates
// ----------------------------------------------------------

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"

#include "cuke_vision/cukeDetector.hpp"

#include <iostream>
#include <stdio.h>

class stereoCamNode {

    public:
        
        // Constructor
        stereoCamNode();

        // Destructor
        // ~stereoCamNode();

    private:
        
        // Node variables
        ros::NodeHandle nH;
        image_transport::ImageTransport it;

        const std::string nodeName = "stereoCamNode";

        // const std::string cameraInfoTopic = "/camera/aligned_depth_to_color/camera_info";
        const std::string cameraInfoTopic = "/camera/depth/camera_info";
        const std::string colorImageTopic = "/camera/color/image_raw";
        // const std::string depthImageTopic = "/camera/aligned_depth_to_color/image_raw";
        const std::string depthImageTopic = "/camera/depth/image_rect_raw";

        // Image transport subscriber
        image_transport::SubscriberFilter colorImageSub;
        image_transport::SubscriberFilter depthImageSub;
        ros::Publisher markerPub;

        // Sync policy for synchronization
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

        // Define synchronizer for callbacks
        message_filters::Synchronizer<SyncPolicy> sync;
        
        // Cucumber detector object
        cukeDetector detector;

        // Current frame and bounding boxes around cukes
        cv::Mat colorFrame;
        cv::Mat depthFrame;
        std::vector<cv::Rect> boxes;

        // Marker points for bounding boxes
        visualization_msgs::Marker points;

        // Camera intrinsics/extrinsics
        boost::shared_ptr<const sensor_msgs::CameraInfo> camInfoPtr;
        float K[9];

        void cameraSetup();
        void imageCallback(const sensor_msgs::ImageConstPtr &colorImageMsg, const sensor_msgs::ImageConstPtr &depthImageMsg);
        void draw3DBounding(cv::Rect bounding);
        void compute3DPoint(const float pixel_x, const float pixel_y, float depth, float (&point)[3]);
        void sendMarkers();
};
