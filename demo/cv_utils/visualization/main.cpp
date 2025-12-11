//
// Created by mfy on 2025/12/11.
//
#include <spdlog/spdlog.h>
#include "cv_utils/visualization/cv_utils_visualization.hpp"
using namespace cv_utils::visualization;
int main() {
    string data_path = std::string(DEMO_PATH) + "data/image_1.png";

    SPDLOG_INFO("data_path : {}",data_path);
    cv::Mat img = cv::imread(data_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " <<data_path  << "\n";
        return 1;
    }
    showMat(img,data_path);
}