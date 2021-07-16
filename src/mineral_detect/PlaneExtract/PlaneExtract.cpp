#include "PlaneExtract.h"

void PlaneExtract::getPlane(Mat &depth_mat, Mat &mask, Mat &cameraMatrix)
{
    cv::rgbd::depthTo3d(depth_mat, cameraMatrix, mat3d);
    auto normals_compute = cv::rgbd::RgbdNormals::create(mat3d.cols, mat3d.rows, CV_32F, cameraMatrix, 1);
    (*normals_compute)(mat3d, normals_mat);
    mineral_plane(mat3d, normals_mat, mask, plane_normals);
}

void PlaneExtract::colorMask(Mat &mask, Mat &colorMask)
{
    RNG rng;
    Vec3b colors[256];
    for (int i = 0; i < 256; i++)
    {
        colors[i] = Vec3b(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    }

    parallel_for_(Range(0, mask.rows * mask.cols), [&](const Range &range)
                  {
                      for (int r = range.start; r < range.end; r++)
                      {
                          int i = r / mask.cols;
                          int j = r % mask.cols;
                          colorMask.ptr<Vec3b>(i)[j] = colors[mask.ptr<uchar>(i)[j]];
                      }
                  });
}

int PlaneExtract::getMaxPlane(Mat &mask)
{
    Mat hist_mat;
    int histSize = 255;
    float range[] = {0, 255};
    const float *histRange = {range};
    bool uniform = true;
    bool accumulate = false;

    calcHist(&mask, 1, 0, Mat(), hist_mat, 1, &histSize, &histRange, uniform, accumulate); //计算直方图，找最大值
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    minMaxLoc(hist_mat, &minVal, &maxVal, &minLoc, &maxLoc);

    inRange(mask, Scalar(maxLoc.y), Scalar(maxLoc.y), mask);
    return maxLoc.y;
}

Point PlaneExtract::getPlaneCentroid(Mat &mask)
{
    Mat kerl0 = getStructuringElement(1, Size(11, 11));
    Mat kerl1 = getStructuringElement(1, Size(5, 5));
    // erode(mask, mask, kerl1);
    dilate(mask, mask, kerl0);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point> centeroid_points;

    for (int i = 0; i < contours.size(); i++)
    {

        cv::Moments centroid = cv::moments(contours[i]);
        cv::Point center_p(centroid.m10 / centroid.m00, centroid.m01 / centroid.m00);
        if (center_p.x <= 0 || center_p.y <= 0)
        {
            continue;
        }
        cout << center_p << endl;
        centeroid_points.push_back(center_p);
    }

    cv::Point center_p;

    if (centeroid_points.size() == 1)
    {
        center_p = centeroid_points[0];
    }
    else
    {
        cv::Moments centroid = cv::moments(centeroid_points);
        center_p = Point(centroid.m10 / centroid.m00, centroid.m01 / centroid.m00);
    }

    return center_p;
}