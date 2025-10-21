#include "img_manager/ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

const std::unordered_map<std::string, std::function<cv::Mat(cv::Mat)> > ImgProcUtil::ARGS_TO_OP = {
    {"TEST",                &ImgProcUtil::proc_test},

    {"CONVERT_BGR2HSV",     &ImgProcUtil::proc_convert_bgr2hsv},
    {"CONVERT_HSV2BGR",     &ImgProcUtil::proc_convert_hsv2bgr},
    {"CONVERT_BGR2GRAY",    &ImgProcUtil::proc_convert_bgr2gray},

    {"DENOISE_GAUSS",       &ImgProcUtil::proc_denoising_gauss_blur},
    {"DENOISE_MEDIAN",      &ImgProcUtil::proc_denoising_median_blur},
    {"DENOISE_BILATERAL",   &ImgProcUtil::proc_denoising_bilateral_fliter},

    {"MORPH_DILATE",        &ImgProcUtil::proc_morph_dilate},
    {"MORPH_ERODE",         &ImgProcUtil::proc_morph_erode},
    {"MORPH_OPEN",          &ImgProcUtil::proc_morph_open},
    {"MORPH_CLOSE",         &ImgProcUtil::proc_morph_close},
    {"MORPH_GRADIENT",      &ImgProcUtil::proc_morph_gradient},

    {"MASK_BLUE",           &ImgProcUtil::proc_mask_blue},

    {"EDGE_SOBEL",          &ImgProcUtil::proc_edge_sobel},
    {"EDGE_SCHARR",         &ImgProcUtil::proc_edge_scharr},
    {"EDGE_LAPLACIAN",      &ImgProcUtil::proc_edge_laplacian},
    {"EDGE_CANNY",          &ImgProcUtil::proc_edge_canny},
};

cv::Mat ImgProcUtil::proc_img(cv::Mat img, std::string arg){
    cv::Mat proc_img;
    try {
        proc_img = ARGS_TO_OP.at(arg)(img);
    } catch (std::out_of_range &e){
        if(arg != "NO_PROC"){
            std::cout << "警告：无法识别的操作参数代号 \"" << arg << "\"。已跳过图像处理操作。\n";
        }
        else {
            std::cout << "警告：已跳过图像处理操作。\n";
        }
        return img;
    }
    return proc_img;
}

cv::Mat ImgProcUtil::proc_test(cv::Mat img){
    cv::Mat original_img = img.clone();
    cv::Mat hsv_img = proc_convert_bgr2hsv(img);
    hsv_img = proc_denoising_bilateral_fliter(hsv_img);
    cv::Mat mask = proc_mask_blue(hsv_img);
    cv::Mat result_img;
    cv::bitwise_and(original_img, original_img, result_img, mask);
    // cv::Mat result_gray = proc_convert_bgr2gray(result_img);
    // mask = proc_edge_canny(result_gray);
    // cv::bitwise_and(original_img, original_img, result_img, mask);
    return result_img;
}

cv::Mat ImgProcUtil::proc_convert_bgr2hsv(cv::Mat img){
    cv::Mat proc_img;
    cv::cvtColor(img, proc_img, cv::COLOR_BGR2HSV);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_convert_hsv2bgr(cv::Mat img){
    cv::Mat proc_img;
    cv::cvtColor(img, proc_img, cv::COLOR_HSV2BGR);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_convert_bgr2gray(cv::Mat img){
    cv::Mat proc_img;
    cv::cvtColor(img, proc_img, cv::COLOR_BGR2GRAY);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_denoising_gauss_blur(cv::Mat img){
    cv::Mat proc_img;
    cv::GaussianBlur(img, proc_img, cv::Size(5, 5), 0, 0);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_denoising_median_blur(cv::Mat img){
    cv::Mat proc_img;
    cv::medianBlur(img, proc_img, 5);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_denoising_bilateral_fliter(cv::Mat img){
    cv::Mat proc_img;
    cv::bilateralFilter(img, proc_img, 9, 75, 75);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_morph_dilate(cv::Mat img){
    cv::Mat proc_img;
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(img, proc_img, cv::MORPH_DILATE, kernal);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_morph_erode(cv::Mat img){
    cv::Mat proc_img;
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(img, proc_img, cv::MORPH_ERODE, kernal);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_morph_open(cv::Mat img){
    cv::Mat proc_img;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(img, proc_img, cv::MORPH_OPEN, kernel);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_morph_close(cv::Mat img){
    cv::Mat proc_img;
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(img, proc_img, cv::MORPH_CLOSE, kernal);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_morph_gradient(cv::Mat img){
    cv::Mat proc_img;
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(img, proc_img, cv::MORPH_GRADIENT, kernal);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_mask_blue(cv::Mat img){
    cv::Mat proc_img;
    // HSV 三通道在 OpenCV 中最大值分别为 180, 255, 255
    // H 代表颜色, 0/180 -> 红色, 60 -> 绿色, 120 -> 蓝色
    // S 表示饱和度, 0 -> 褪色, 255 -> 纯色
    // V 表示亮度, 0 -> 黑色, 255 -> 亮色
    cv::Scalar lower_mask = cv::Scalar(90, 33, 45);
    cv::Scalar upper_mask = cv::Scalar(150, 255, 255);
    cv::inRange(img, lower_mask, upper_mask, proc_img);
    return proc_img;
}

cv::Mat ImgProcUtil::proc_edge_sobel(cv::Mat img){
    cv::Mat grad_x, grad_y, dst;
    cv::Sobel(img, grad_x, CV_16S, 1, 0, 5);
    cv::Sobel(img, grad_y, CV_16S, 0, 1, 5);
    cv::convertScaleAbs(grad_x, grad_x);
    cv::convertScaleAbs(grad_y, grad_y);
    cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, dst);
    return dst;
}

cv::Mat ImgProcUtil::proc_edge_scharr(cv::Mat img){
    cv::Mat grad_x, grad_y, dst;
    cv::Scharr(img, grad_x, CV_16S, 1, 0);
    cv::Scharr(img, grad_y, CV_16S, 0, 1);
    cv::convertScaleAbs(grad_x, grad_x);
    cv::convertScaleAbs(grad_y, grad_y);
    cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, dst);
    return dst;
}

cv::Mat ImgProcUtil::proc_edge_laplacian(cv::Mat img){
    cv::Mat grad;
    cv::Laplacian(img, grad, CV_16S, 3);
    cv::convertScaleAbs(grad, grad);
    return grad;
}

cv::Mat ImgProcUtil::proc_edge_canny(cv::Mat img){
    cv::Mat grad;
    cv::Canny(img, grad, 60, 140);
    return grad;
}