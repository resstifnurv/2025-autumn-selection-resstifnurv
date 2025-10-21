#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <functional>
#include <unordered_map>

#define DEFAULT_MAT cv::Mat::zeros(img.size(), CV_8U)
#define DEFAULT_MAT_(type) cv::Mat::zeros(img.size(), type)

class ImgProcUtil{
private:
    ImgProcUtil() = default;
    ~ImgProcUtil() = delete;
    ImgProcUtil& operator=(const ImgProcUtil&) = delete;
    static const std::unordered_map<std::string, std::function<cv::Mat(cv::Mat)> > ARGS_TO_OP;

    static cv::Mat proc_test(cv::Mat img);                          // arg = "TEST"

    static cv::Mat proc_convert_bgr2hsv(cv::Mat img);               // arg = "CONVERT_BGR2HSV"
    static cv::Mat proc_convert_hsv2bgr(cv::Mat img);               // arg = "CONVERT_HSV2BGR"
    static cv::Mat proc_convert_bgr2gray(cv::Mat img);              // arg = "CONVERT_BGR2GRAY"

    static cv::Mat proc_denoising_gauss_blur(cv::Mat img);          // arg = "DENOISE_GAUSS"
    static cv::Mat proc_denoising_median_blur(cv::Mat img);         // arg = "DENOISE_MEDIAN"
    static cv::Mat proc_denoising_bilateral_fliter(cv::Mat img);    // arg = "DENOISE_BILATERAL"

    static cv::Mat proc_morph_dilate(cv::Mat img);                  // arg = "MORPH_DILATE"
    static cv::Mat proc_morph_erode(cv::Mat img);                   // arg = "MORPH_ERODE"
    static cv::Mat proc_morph_open(cv::Mat img);                    // arg = "MORPH_OPEN"
    static cv::Mat proc_morph_close(cv::Mat img);                   // arg = "MORPH_CLOSE"
    static cv::Mat proc_morph_gradient(cv::Mat img);                // arg = "MORPH_GRADIENT"

    static cv::Mat proc_mask_blue(cv::Mat img);                     // arg = "MASK_BLUE"

    static cv::Mat proc_edge_sobel(cv::Mat img);                    // arg = "EDGE_SOBEL"
    static cv::Mat proc_edge_scharr(cv::Mat img);                   // arg = "EDGE_SCHARR"
    static cv::Mat proc_edge_laplacian(cv::Mat img);                // arg = "EDGE_LAPLACIAN"
    static cv::Mat proc_edge_canny(cv::Mat img);                    // arg = "EDGE_CANNY"

public:
    static cv::Mat proc_img(cv::Mat img, std::string arg);
};
