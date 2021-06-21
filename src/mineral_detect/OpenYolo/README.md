# OpenYolo

用OpenCV部署Yolo3及Yolo3-tiny模型

## 配置环境
>  Ubuntu 20.04 

> OpenCV 4.4

> Cmake 3.16.3

## 使用教程
在你需要使用的工程下使用此工程为submodule（得先初始化本地仓库）
> git submodule add https://git.scutbot.cn/hzx/OpenYolo.git

这时候工程下会出现OpenYolo文件夹，将[模型参数](https://pan.scutbot.cn/s/i5FsTFNJADrJfcX)和[cfg文件](https://pan.scutbot.cn/s/rbADybrNSfws42M)目录写进代码，编写CMakeLists.txt链接detect库即可

```cpp
cmake_minimum_required(VERSION 3.0.0)
project(main VERSION 0.1.0)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV REQUIRED)

add_subdirectory(OpenYolo/detect)
add_executable(main main.cpp)
target_link_libraries(main detect)
```
## 示例
```cpp
#include "detect.h"

int main(int argc, char **argv)
{
    Object yolo_out;
    detect yolo("../yolov3-tiny.cfg", "../yolov3-tiny_final.weights", "../config/camPara.yml"); //加载模型参数
    VideoCapture cap(0);
    Mat frame, blob;

    while (waitKey(1) < 0)
    {
        cap >> frame;
        yolo_out = yolo.inference(frame); //推理模型得到结果，结果中包含每个目标的类别、矩形框及置信度

        if (yolo_out.boxes.size() > 0) //输出第一个目标的类别及左上角的坐标
        {
            cout << yolo.classes[yolo_out.classIds[0]] << ": " << yolo_out.boxes[0].tl() << endl;
        }
        imshow("frame", frame);
        waitKey(1);
    }
}
```
