//
// Created by mfy on 2025/12/11.
//

#ifndef VISIONUTILS_CV_UTILS_VISUALIZATION_HPP
#define VISIONUTILS_CV_UTILS_VISUALIZATION_HPP
#include <opencv2/opencv.hpp>
using namespace std;
namespace cv_utils::visualization {
    void showMat(const cv::Mat & image,const string & windows_name) {
        cv::namedWindow(windows_name,cv::WINDOW_FREERATIO);
        cv::imshow(windows_name,image);
        cv::waitKey(0);
    }
}
#endif //VISIONUTILS_CV_UTILS_VISUALIZATION_HPP