// ----------------------------------------------------------
// NAME: Realsense Node Header
// DESCRIPTION: Node that replaces the realsense_ros package
// ----------------------------------------------------------
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include "cuke_vision/cukeDetector.hpp"
#include <algorithm>
#include <string>
#include <iostream>
#include <cmath>


class realsenseNode {


    public:

    private:
        
        // RS2 config 
        rs2::pipeline pipeline;
        rs2::config config;

        // RS2 frames
        rs2::frameset;
        rs2::alignToDepth;

        // Mutex
        std::mutex mut;

        // RS2 pointclouds
        rs2::pointcloud pointCloud;
        rs2::points points;

        // Constants
        const imageHeight = 480;
        const imageWidth = 640;



};
