#include "img_manager/VirtualWriter.hpp"
#include <exception>
#include <opencv2/videoio.hpp>
#include <signal.h>

std::map<std::string, int> VirtualWriter::FOURCC = {
    {"avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D')},
    {"mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1')},
    {"mkv", cv::VideoWriter::fourcc('X', '2', '6', '4')}
};

const int VirtualWriter::DEFAULT_FOURCC = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

VirtualWriter::VirtualWriter(std::string target_path, int fps, cv::Size frame_size)
: target_path(target_path), fps(fps), frame_size(frame_size){
    std::string end_with = target_path.substr(
        target_path.find_last_of('.') + 1
    );
    int fourcc = DEFAULT_FOURCC;
    if(FOURCC.find(end_with) != FOURCC.end()){
        fourcc = FOURCC[end_with];
    }

    try{
        this->writer.open(
            this->target_path,
            fourcc,
            this->fps,
            this->frame_size
        );
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    if(!this->writer.isOpened()){
        std::cout << "错误：无法打开 VideoWriter\n";
        raise(SIGINT);
    }
}

VirtualWriter::~VirtualWriter(){
    this->writer.release();
}

void VirtualWriter::write(const cv::Mat& image){
    this->writer.write(image);
}