#include "detect.h"

/**
 * @brief 画图函数，画出检测出的目标框
 */
void detect::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame)
{

    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

    std::string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ": " + label;
    }

    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - labelSize.height),
              Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}

/**
 * @brief 后处理，对yolov3的推理结果进行处理得到想要的数据，返回值是我们自定义的结构体Object
 */
Object detect::postprocess(Mat &frame, const std::vector<Mat> &outs, Net &net)
{
    //设置计算后端所用，此处不用理会
    int backend = 0;
    //返回最后一层，即输出层的索引
    static std::vector<int> outLayers = net.getUnconnectedOutLayers();
    //获得输出层的名称
    static std::string outLayerType = net.getLayer(outLayers[0])->type;

    //推理出的物体类型的ID
    std::vector<int> classIds;
    //推理出的物体的置信度
    std::vector<float> confidences;
    //推理出的物体的目标框
    std::vector<Rect> boxes;

    //如果输出层类型是“DetectionOutput”，我们用的Yolov3就是这个类型，代表Anchor-base
    if (outLayerType == "DetectionOutput")
    {
        //如果推理出的结果不为空
        CV_Assert(outs.size() > 0);
        //遍历所有特征图的输出
        for (size_t k = 0; k < outs.size(); k++)
        {
            //获得第k个特征图的输出的具体数值
            float *data = (float *)outs[k].data;
            //遍历第k个输出的特征图的所有网格，7表示每个网格有7个属性：[batchId, classId, confidence, left, top, right, bottom]
            for (size_t i = 0; i < outs[k].total(); i += 7)
            {
                //获得k特征图i网格的置信度
                float confidence = data[i + 2];
                //若置信度超过阈值
                if (confidence > confThreshold)
                {
                    //获得k特征图i网格的目标框的左上顶点的x坐标，此处的值是除了长度的
                    int left = (int)data[i + 3];
                    //获得k特征图i网格的目标框的左上顶点的y坐标，此处的值是除了宽度的
                    int top = (int)data[i + 4];
                    //获得k特征图i网格的目标框的右下顶点的x坐标，此处的值是除了长度的
                    int right = (int)data[i + 5];
                    //获得k特征图i网格的目标框的右下顶点的y坐标，此处的值是除了宽度的
                    int bottom = (int)data[i + 6];
                    //获得k特征图i网格的目标框的宽
                    int width = right - left + 1;
                    //获得k特征图i网格的目标框的长
                    int height = bottom - top + 1;
                    //此处是保证目标框长宽不超过整张图像，i think上面6个数都是为这个判断服务
                    if (width <= 2 || height <= 2)
                    {
                        //下面6个数是真实的左上，右下顶点坐标及长宽，单位是像素
                        left = (int)(data[i + 3] * frame.cols);
                        top = (int)(data[i + 4] * frame.rows);
                        right = (int)(data[i + 5] * frame.cols);
                        bottom = (int)(data[i + 6] * frame.rows);
                        width = right - left + 1;
                        height = bottom - top + 1;
                    }
                    //保存该k特征图i网格的目标框的类型ID，注意跳过了第一个表示背景的ID
                    classIds.push_back((int)(data[i + 1]) - 1);
                    //保存该k特征图i网格的目标框，使用Opencv的Rect类型
                    boxes.push_back(Rect(left, top, width, height));
                    //保存该k特征图i网格的目标框的置信度
                    confidences.push_back(confidence);
                }
            }
        }
    }

    //如果输出层的名称是"Region"，这应该是anchor-free类算法的输出，代表网络：CenterNet，此处我们不用这个
    else if (outLayerType == "Region")
    {
        for (size_t i = 0; i < outs.size(); ++i)
        {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            float *data = (float *)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
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
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }
    }

    //如果没找到对应的输出层的类型，抛出错误
    else
        CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

    //如果输出层的数量大于1，则需要NMS
    //NMS在Region层中只在DNN_BACKEND_OPENCV上使用
    //以下是NMS的实现，感兴趣可以参考一下，最后的结果是每个网格输出一个最优的Bounding Box，最后再赋值给classIds，confidences和boxes
    if (outLayers.size() > 1 || (outLayerType == "Region" && backend != DNN_BACKEND_OPENCV))
    {
        std::map<int, std::vector<size_t>> class2indices;
        for (size_t i = 0; i < classIds.size(); i++)
        {
            if (confidences[i] >= confThreshold)
            {
                class2indices[classIds[i]].push_back(i);
            }
        }
        std::vector<Rect> nmsBoxes;
        std::vector<float> nmsConfidences;
        std::vector<int> nmsClassIds;
        for (std::map<int, std::vector<size_t>>::iterator it = class2indices.begin(); it != class2indices.end(); ++it)
        {
            std::vector<Rect> localBoxes;
            std::vector<float> localConfidences;
            std::vector<size_t> classIndices = it->second;
            for (size_t i = 0; i < classIndices.size(); i++)
            {
                localBoxes.push_back(boxes[classIndices[i]]);
                localConfidences.push_back(confidences[classIndices[i]]);
            }
            std::vector<int> nmsIndices;
            NMSBoxes(localBoxes, localConfidences, confThreshold, nmsThreshold, nmsIndices);
            for (size_t i = 0; i < nmsIndices.size(); i++)
            {
                size_t idx = nmsIndices[i];
                nmsBoxes.push_back(localBoxes[idx]);
                nmsConfidences.push_back(localConfidences[idx]);
                nmsClassIds.push_back(it->first);
            }
        }
        boxes = nmsBoxes;
        classIds = nmsClassIds;
        confidences = nmsConfidences;
    }

    //遍历所有经过NMS后的目标框
    for (size_t idx = 0; idx < boxes.size(); ++idx)
    {
        //在图中画出所有目标框
        circle(frame, Point((boxes[0].br() + boxes[0].tl()) / 2), 2, Scalar(0, 0, 255),2);
        Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }

    //将boxes，classIds，confidences保存在我们自己定义的结构体Object中，作为返回值供外部调用
    Object res;
    res.boxes = boxes;
    res.classIds = classIds;
    res.confidences = confidences;
    return res;
}

/**
* @brief yolov3推理接口，供外部调用，返回值是自定义的Object
*/
Object detect::inference(Mat &frame)
{
    //图像预处理
    preprocess(frame, yolo, Size(inpWidth, inpHeight));
    //创建一个Mat用来临时保存正向推理的结果
    std::vector<Mat> outs;
    //yolo正向推理，获得推理结果
    yolo.forward(outs, outNames);
    return postprocess(frame, outs, yolo);
}

detect::detect(string cfg_path, string model_path, string camPara)
{
    //读取存储在Darknet模型文件中的网络模型
    yolo = cv::dnn::readNetFromDarknet(cfg_path, model_path);
    //返回最后一层，即输出层的名称
    outNames = yolo.getUnconnectedOutLayersNames();
    //要求网络使用它所支持的特定计算后端，此处为默认，如果是openvino的opencv的话就是特定的引擎
    yolo.setPreferableBackend(backend);
    //要求网络在特定的目标设备上进行计算，此处为CPU
    yolo.setPreferableTarget(0);

    //读取相机内参及畸变，用于下面计算角度
    cv::FileStorage fs(camPara, cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeff;
}

/**
* @brief 计算出相机坐标系下center_point的相对角
*/
float detect::cal_angle(Point center_point)
{
    //图像坐标系下的齐次坐标
    cv::Mat tf_point(3, 1, CV_32F);
    //相机内参转置
    cv::Mat cameraMatrix_inverse;
    //计算内参转置
    cameraMatrix.convertTo(cameraMatrix_inverse, CV_32F);
    cameraMatrix_inverse = cameraMatrix_inverse.inv();

    //图像坐标系下的坐标
    cv::Point true_point = cv::Point2f((float)center_point.x, (float)center_point.y);

    //给图像坐标系下的齐次坐标赋值
    tf_point.at<float>(0) = center_point.x;
    tf_point.at<float>(1) = center_point.y;
    tf_point.at<float>(2) = 1;
    //计算相机坐标系下的坐标
    cv::Mat tf_result = cameraMatrix_inverse * tf_point;
    //计算相对角
    cv::Point2f relative_angle = cv::Point2f(atan(tf_result.at<float>(0)) / 3.14 * 180, atan(tf_result.at<float>(1)) / 3.14 * 180);
    return relative_angle.x;
}