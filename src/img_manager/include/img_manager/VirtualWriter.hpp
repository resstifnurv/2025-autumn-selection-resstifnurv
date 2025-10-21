#include <unordered_map>
#include <opencv2/opencv.hpp>

class VirtualWriter{
public:
    VirtualWriter(std::string target_path, int fps, cv::Size frame_size);
    ~VirtualWriter();
    void write(const cv::Mat& image);

private:
    static const std::unordered_map<std::string, int> FOURCC;
    static const int DEFAULT_FOURCC;
    std::string target_path;
    int fps;
    cv::Size frame_size;
    cv::VideoWriter writer;
};