//
// Created by dhp on 5/05/19.
//

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

/*此文件中导入了常用的开源库的头文件，以简化其他.h和.cpp文件中的头文件导入*/
//Eigen库
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;
//Sophus库
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

//OpenCV库
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;
//STL
#include <vector>
#include <list>
//智能指针相关
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

#include <thread>
using namespace std;

#endif
