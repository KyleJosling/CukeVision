// ----------------------------------------------------------
// NAME: Cuke Detector
// DESCRIPTION: Neural network interface for cucumber detection from an image
// ----------------------------------------------------------

#include "cuke_vision/cukeDetector.hpp"

// Constructor
cukeDetector::cukeDetector() {

    // Load classes
    std::string line;
    std::ifstream ifs(classesFile.c_str());
    while (getline(ifs, line)) classes.push_back(line);

    // Load the network
    net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);

    // Default backend should be OpenCV
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    #ifdef GUI
    namedWindow(kWinName, cv::WINDOW_NORMAL);
    #endif
}

// Destructor
cukeDetector::~cukeDetector() {

    #ifdef GUI
    cv::destroyWindow(kWinName);
    #endif
}

// Cucumber detection function
std::vector<cv::Rect> cukeDetector::detectCukes( cv::Mat &frame) {

    // Create a 4D blob from a frame.
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cvSize(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    std::vector<cv::Mat> outs;
    std::vector <cv::String> namez;
    for (int i = 0; i < (namez.size()); i++) {
        std::cout << namez[i] << std::endl;
    }
    net.forward(outs, getOutputsNames(net));

    // Remove the bounding boxes with low confidence, publish message
    std::vector<cv::Rect> boxes;
    postprocess(frame, outs, boxes);

    // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time for a frame : %.2f ms", t);

    #ifdef GUI
    cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
    cv::imshow(kWinName, frame);
    cv::waitKey(30);
    #endif

}

// Draws bounding boxes around detected cucumbers over confidence threshold
void cukeDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame) {

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


// Gets output layer names
std::vector<cv::String> cukeDetector::getOutputsNames(const cv::dnn::Net& net) {
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

// Post processing, removes boxes of low confidence
void cukeDetector::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<cv::Rect> &boxes) {

    std::vector<int> classIds;
    std::vector<float> confidences;

    // Message to publish
    cuke_vision::boundingBoxMsg msg;
    
    for (size_t i = 0; i < outs.size(); ++i) {

        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {

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

    #ifdef GUI
    for (size_t i = 0; i < indices.size(); ++i) {

        int idx = indices[i];
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
    #endif
}
