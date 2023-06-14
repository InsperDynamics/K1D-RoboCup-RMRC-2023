#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;
using namespace dnn;

Net net;
vector<string> class_list = {
    "Corrosive", "Dangerous when wet", "Explosive", "Flammable Fluid", "Flammable solid",
    "Infectious substance", "Inhalation hazard", "Miscellaneous", "Non-flammable gas",
    "Organic peroxide", "Oxidizer", "Poison", "Radioactive", "Spontaneously combustile"
};
vector<Scalar> colors = {
    Scalar(255, 255, 0), Scalar(0, 255, 0), Scalar(0, 255, 255), Scalar(255, 0, 0)
};
const int INPUT_WIDTH = 640;
const int INPUT_HEIGHT = 640;
const float SCORE_THRESHOLD = 0.25;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.5;

void detect(cv::Mat& image, cv::dnn::Net& net, cv::Mat& preds) {
    cv::Mat blob = cv::dnn::blobFromImage(image, 1/255.0, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);
    preds = net.forward();
}

void wrap_detection(cv::Mat& input_image, cv::Mat& output_data, std::vector<int>& class_ids, std::vector<float>& confidences, std::vector<cv::Rect>& boxes) {
    int rows = output_data.size[2];
    int image_width = input_image.cols;
    int image_height = input_image.rows;
    float x_factor = static_cast<float>(image_width) / INPUT_WIDTH;
    float y_factor = static_cast<float>(image_height) / INPUT_HEIGHT;
    for (int r = 0; r < rows; ++r) {
        cv::Mat row = output_data.row(r);
        float confidence = row.at<float>(4);
        if (confidence >= 0.4) {
            cv::Mat classes_scores = row.colRange(5, row.cols);
            cv::Point max_indx;
            cv::minMaxLoc(classes_scores, nullptr, nullptr, nullptr, &max_indx);
            int class_id = max_indx.y;
            if (classes_scores.at<float>(class_id) > 0.25) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id);
                float x = row.at<float>(0);
                float y = row.at<float>(1);
                float w = row.at<float>(2);
                float h = row.at<float>(3);
                int left = static_cast<int>((x - 0.5 * w) * x_factor);
                int top = static_cast<int>((y - 0.5 * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);
                boxes.emplace_back(left, top, width, height);
            }
        }
    }
    std::vector<int> indexes;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indexes);
    std::vector<int> result_class_ids;
    std::vector<float> result_confidences;
    std::vector<cv::Rect> result_boxes;
    for (int i : indexes) {
        result_confidences.push_back(confidences[i]);
        result_class_ids.push_back(class_ids[i]);
        result_boxes.push_back(boxes[i]);
    }
    class_ids = result_class_ids;
    confidences = result_confidences;
    boxes = result_boxes;
}

cv::Mat format_yolov5(cv::Mat& frame) {
    int row = frame.rows;
    int col = frame.cols;
    int _max = std::max(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, frame.type());
    frame.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

static void InitializeHazmat()
{
    string modelfile = "best.onnx";
    net = readNetFromONNX(configfile);
}

static Mat DetectHazmat(Mat image)
{
    Mat inputImage = format_yolov5(image);
    Mat outs;
    detect(inputImage, net, outs);
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    wrap_detection(inputImage, outs, class_ids, confidences, boxes);
    for (size_t i = 0; i < class_ids.size(); ++i) {
        int classid = class_ids[i];
        float confidence = confidences[i];
        cv::Rect box = boxes[i];
        cv::Scalar color = colors[classid % colors.size()];
        cv::rectangle(image, box, color, 2);
        cv::rectangle(image, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, -1);
        cv::putText(image, class_list[classid], cv::Point(box.x, box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    return image;
}