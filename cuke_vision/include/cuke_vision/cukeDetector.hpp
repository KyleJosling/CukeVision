// ----------------------------------------------------------
// NAME: Cuke Detector Header
// DESCRIPTION: Neural network interface for cucumber
// detection from an image
// ----------------------------------------------------------

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/tracking/tracking.hpp>

#include <ros/ros.h>

#include "cuke_vision/boundingBoxMsg.h"

class cukeDetector {

    public:

    cukeDetector();

    ~cukeDetector();

    void detectCukes( cv::Mat &frame, std::vector<cv::Rect> &boxes);

    private:

        // Window name
        const std::string kWinName = "Object Detection";

        // Class parameters
        std::vector<std::string> classes;
        std::string classesFile = "/home/kylejosling/Downloads/cuke-training/classes.names";

        // Model configuration files
        cv::String modelConfiguration = "/home/kylejosling/Downloads/cuke-training/darknet-yolov3.cfg";
        cv::String modelWeights = "/home/kylejosling/Downloads/cuke-training/weights/darknet-yolov3_final.weights";

        // Neural network
        cv::dnn::Net net;

        // Network parameters
        int inpWidth = 416;
        int inpHeight = 416;

        float confThreshold = 0.5; // Confidence threshold
        float nmsThreshold = 0.4;  // Non-maximum suppression threshold

        cv::Mat frame;
        cv::Mat blob;

        void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

        std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);

        void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<cv::Rect> &boxes);
};
