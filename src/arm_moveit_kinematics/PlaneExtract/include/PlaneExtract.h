_Pragma("once");
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class PlaneExtract
{
public:
    Mat mat3d;
    Mat normals_mat;
    Mat plane_normals;
    PlaneExtract(){};
    ~PlaneExtract(){};
    void getPlane(Mat &depth_mat, Mat &mask, Mat &cameraMatrix);
    void colorMask(Mat &mask,Mat &colorMask);
    Point getPlaneCentroid(Mat &mask);
    int getMaxPlane(Mat &mask);
private:
    rgbd::RgbdPlane mineral_plane;
};