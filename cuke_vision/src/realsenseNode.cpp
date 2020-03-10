// ----------------------------------------------------------
// ----------------------------------------------------------
// NAME: Realsense Node
// DESCRIPTION: Node that replaces the realsense_ros package
// ----------------------------------------------------------

#include "cuke_vision/realsenseNode.hpp"


realsenseNode::realsenseNode() {

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16 , 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    pipe.start(cfg);

    alignToDepth = new rs2::align(RS2_STREAM_DEPTH);
}

void realsenseNode::loop() {

    ros::Rate r(30);

    while (ros::ok()) {

        frameset = pipe.wait_for_frames();
        frameset = alignToDepth(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        
        // Publish frames in rapid succession
        publishFrame(color);
        publishFrame(depth);


    }

}

sensor_msgs::ImagePtr realsenseNode::convertToSensorMsg(rs2::frame fromFrame) {
    
    cv::Mat image;
    auto bytesPerPixel = 1;
    std::string encoding;
    std::string frameId;

    if (frame.is<rs2::video_frame()) {

        encoding = sensor_msgs::image_encodings::RGB8;
        frameId = "camera_color_optical_frame";
        bytesPerPixel = (frame.as<rs2::video_frame>()).get_bytes_per_pixel();
        image.create(cv::Size(640, 480), CV_8UC3, (void*)fromFrame.get_data(), cv::Mat::AUTO_STEP);

    } else if (frame.is<rs2::depth_frame>) {

        encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        frameId = "camera_color_optical_frame";
        bytesPerPixel = (frame.as<rs2::depth_frame>()).get_bytes_per_pixel();
        image.create(cv::Size(640, 480), CV_16UC1, (void*)fromFrame.get_data(), cv::Mat::AUTO_STEP);

    } else {

    }
    
    sensor_msgs::ImagePtr img;
    img = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
    img->height = imageHeight;
    img->width = imageWidth;
    img->is_bigendian = false;
    img->step = imageWidth*bytesPerPixel;
    img->header.frame_id = frameId;
    img->header.stamp = ;
    // img->header.seq = ;

    return img;
}

void realsenseNode::publishFrame() {


}

auto realsenseNode::objectCallback( const rs2::frame &frame) {
    
    std::lock_guard<std::mutex> lock(mut);

}

int main(int argc, char **argv) {
    
    std::string nodeName = "realsenseNode";
    ros::init(argc, argv, nodeName);

    realsenseNode rsN;

    ros::spin();

    return 0;
}
