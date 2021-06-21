_Pragma("once");
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace dnn;
using namespace std;

struct Object
{
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect> boxes;
};

class detect
{
private:
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame);
    Object postprocess(Mat &frame, const std::vector<Mat> &out, Net &net);
    inline void preprocess(const Mat &frame, Net &net, Size inpSize, float scale = 0.0039,
                           bool swapRB = 1)
    {
        static Mat blob;
        // Create a 4D blob from a frame.
        if (inpSize.width <= 0)
            inpSize.width = frame.cols;
        if (inpSize.height <= 0)
            inpSize.height = frame.rows;
        blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false, CV_8U);

        // Run a model.
        net.setInput(blob, "", scale);
        if (net.getLayer(0)->outputNameToIndex("im_info") != -1) // Faster-RCNN or R-FCN
        {
            resize(frame, frame, inpSize);
            Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
            net.setInput(imInfo, "im_info");
        }
    }

public:
    float confThreshold = 0.1, nmsThreshold = 0.1;
    int inpWidth = 640;
    int inpHeight = 480;
    int backend = 0;
    cv::Mat distCoeff, cameraMatrix;
    cv::dnn::Net yolo;
    std::vector<std::string> classes = {"mineral"};
    std::vector<String> outNames;

    detect(string model_path, string cfg_path, string camPara);
    ~detect(){};
    Object inference(Mat &);
    float cal_angle(Point center_point);
};
