#ifndef ARUCO_CV_VERSIONING
#define  ARUCO_CV_VERSIONING
#include <opencv2/core/core.hpp>
#if  CV_MAJOR_VERSION >= 4
#include <opencv2/imgproc.hpp>

#define CV_BGR2GRAY cv::COLOR_BGR2GRAY
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR
#define CV_FONT_HERSHEY_COMPLEX cv::FONT_HERSHEY_COMPLEX
#endif

#endif

