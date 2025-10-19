#pragma once

#include <iostream>
#include <signal.h>
#include <opencv2/opencv.hpp>

class VirtualCamera{

public:
    VirtualCamera(std::string source_path);
    ~VirtualCamera();
    int open(void);
    int close(void);
    cv::Mat get_frame(void);
    
private:
    std::string source_path;
    static const std::set<std::string> PICTURE_TYPE;
    static const std::set<std::string> VIDEO_TYPE;
    std::string open_type;
    cv::VideoCapture cam;
    bool is_opened;
};