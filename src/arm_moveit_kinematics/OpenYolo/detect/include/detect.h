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
    /**
     * @brief 画图函数，画出检测出的目标框
     */
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame);
    /**
     * @brief 后处理，对yolov3的推理结果进行处理得到想要的数据，返回值是我们自定义的结构体Object
     */
    Object postprocess(Mat &frame, const std::vector<Mat> &out, Net &net);
    /**
     * @brief 图像预处理
     */
    inline void preprocess(const Mat &frame, Net &net, Size inpSize, float scale = 0.0039,
                           bool swapRB = 1)
    {
        static Mat blob;
        // 从frame中创建一个4维的blob，维度分别是NCHW
        if (inpSize.width <= 0)
            inpSize.width = frame.cols;
        if (inpSize.height <= 0)
            inpSize.height = frame.rows;
        blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false, CV_8U);

        // 设置网络的新输入值.
        net.setInput(blob, "", scale);
        // 如果是Faster-RCNN or R-FCN，在RPN网络要变为(M/16)x(N/16)，所以在im_info层要设置一下
        if (net.getLayer(0)->outputNameToIndex("im_info") != -1) // Faster-RCNN or R-FCN
        {
            resize(frame, frame, inpSize);
            Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
            net.setInput(imInfo, "im_info");
        }
    }

public:
    //置信度，nms阈值
    float confThreshold = 0.1, nmsThreshold = 0.1;
    //图像长度
    int inpWidth = 640;
    //图像宽度
    int inpHeight = 480;
    //设置计算后端为默认
    int backend = 0;
    //相机内参，畸变
    cv::Mat distCoeff, cameraMatrix;
    //实例化dnn的Net
    cv::dnn::Net yolo;
    //推理结果的类型名
    std::vector<std::string> classes = {"mineral"};
    //输出层的名称
    std::vector<String> outNames;

    //构造函数
    detect(string model_path, string cfg_path, string camPara);
    //析构函数
    ~detect(){};
    /**
    * @brief yolov3推理接口，供外部调用，返回值是自定义的Object
    */
    Object inference(Mat &);
    /**
    * @brief 计算出相机坐标系下center_point的相对角
    */
    float cal_angle(Point center_point);
};
