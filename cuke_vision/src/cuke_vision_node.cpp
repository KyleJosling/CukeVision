#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/tracking/tracking.hpp>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <stdio.h>

#include "cuke_vision/detectObject.hpp"


class ardrone_object_tracker {

    public:

        // Initialization list
        ardrone_object_tracker():

        // Initialize image transport instance
        it(nH) { 

        }

        void imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {

        }
        
    private:
        const std::string arImageTopic = "/ardrone/front/image_raw";
        const std::string objectTopic = "/object";

        ros::NodeHandle nH;
        image_transport::ImageTransport it;

        // Image transport subscriber
        image_transport::Subscriber arImageSub;

        // Publisher
        ros::Publisher objectPub;

        // CV variables
        bool ok;
        int hue, sat, val, width;

        cv::Mat frame;
        cv::Rect2d roi;
        cv::Ptr<cv::TrackerKCF> tracker;

};

const char* keys =
"{help h usage ? | | Usage examples: \n\t\t./object_detection_yolo.out --image=dog.jpg \n\t\t./object_detection_yolo.out --video=run_sm.mp4}"
"{image i        |<none>| input image   }"
"{video v       |<none>| input video   }";

// Neural network parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;  // Width of network's input image
int inpHeight = 416; // Height of network's input image
std::vector<std::string> classes;

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& out);

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);

int main(int argc, char** argv) {

    std::cout << "CV_major : " << CV_MAJOR_VERSION << std::endl;
    std::cout << "CV_minor : " << CV_MINOR_VERSION << std::endl;
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Use this script to run object detection using YOLO3 in OpenCV.");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    // Load names of classes
    std::string classesFile = "/home/kylejosling/Downloads/test/ObjectDetection-YOLO/coco.names";
    std::ifstream ifs(classesFile.c_str());
    std::string line;
    while (getline(ifs, line)) classes.push_back(line);

    // Give the configuration and weight files for the model
    cv::String modelConfiguration = "/home/kylejosling/Downloads/test/ObjectDetection-YOLO/yolov3.cfg";
    cv::String modelWeights = "/home/kylejosling/Downloads/test/ObjectDetection-YOLO/yolov3.weights";

    // Load the network
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);

    // Default backend should be OpenCV
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Open a video file or an image file or a camera stream.
    std::string str, outputFile;
    cv::VideoCapture cap;
    cv::VideoWriter video;
    cv::Mat frame, blob;

    try {
        
        outputFile = "yolo_out_cpp.avi";
        if (parser.has("image"))
        {
            // Open the image file
            str = parser.get<cv::String>("image");
            std::ifstream ifile(str);
            if (!ifile) throw("error");
            cap.open(str);
            str.replace(str.end()-4, str.end(), "_yolo_out_cpp.jpg");
            outputFile = str;
        }
        else if (parser.has("video"))
        {
            // Open the video file
            str = parser.get<cv::String>("video");
            std::ifstream ifile(str);
            if (!ifile) throw("error");
            cap.open(str);
            str.replace(str.end()-4, str.end(), "_yolo_out_cpp.avi");
            outputFile = str;
        }
        // Open the webcaom
        else cap.open(0);
        
    }
    catch(...) {
        std::cout << "Could not open the input image/video stream" << std::endl;
        return 0;
    }


    // Get the video writer initialized to save the output video
    if (!parser.has("image")) {
        video.open(outputFile, cv::VideoWriter::fourcc('M','J','P','G'), 28, cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
    }

    // Create a window
    static const std::string kWinName = "Deep learning object detection in OpenCV";
    namedWindow(kWinName, cv::WINDOW_NORMAL);

    // Process frames.
    while (cv::waitKey(1) < 0)
    {
        // get frame from the video
        cap >> frame;

        // Stop the program if reached end of video
        if (frame.empty()) {
            std::cout << "Done processing !!!" << std::endl;
            std::cout << "Output file is stored as " << outputFile << std::endl;
            cv::waitKey(3000);
            break;
        }

        // Create a 4D blob from a frame.
        cv::dnn::blobFromImage(frame, blob, 1/255.0, cvSize(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

        //Sets the input to the network
        net.setInput(blob);

        // Runs the forward pass to get output of the output layers
        std::vector<cv::Mat> outs;
        // TODO is this right? old function was causing linker errors 
        net.forward(outs, getOutputsNames(net));

        // Remove the bounding boxes with low confidence
        postprocess(frame, outs);

        // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        std::vector<double> layersTimes;
        double freq = cv::getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        std::string label = cv::format("Inference time for a frame : %.2f ms", t);
        cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        
        // Write the frame with the detection boxes
        cv::Mat detectedFrame;
        frame.convertTo(detectedFrame, CV_8U);
        if (parser.has("image")) imwrite(outputFile, detectedFrame);
        else video.write(detectedFrame);
        
        cv::imshow(kWinName, frame);

    }
    // std::string nodeName = "cuke_vision_node";
    // ros::init(argc, argv, nodeName);

    // ardrone_object_tracker ar;

    return 0;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs)
{
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
}

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    
    //Get the label for the class name and its confidence
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    } else {
        std::cout << "Classes r empty" << std::endl;
    }
    
    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0),1);
}

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        
        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
