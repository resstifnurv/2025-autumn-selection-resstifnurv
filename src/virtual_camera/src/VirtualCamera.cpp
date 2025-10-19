#include <exception>
#include <opencv2/videoio.hpp>
#include <virtual_camera/VirtualCamera.hpp>

const std::set<std::string> VirtualCamera::PICTURE_TYPE = {
    "jpg", "jpeg", "png"
};
const std::set<std::string> VirtualCamera::VIDEO_TYPE = {
    "avi", "mp4"
};

VirtualCamera::VirtualCamera(std::string source_path)
: source_path(source_path){
    std::string end_with = source_path.substr(
        source_path.find_last_of('.') + 1
    );
    if(PICTURE_TYPE.find(end_with) != PICTURE_TYPE.end()){
        this->open_type = "PICTURE";
    }
    else if(VIDEO_TYPE.find(end_with) != VIDEO_TYPE.end()){
        this->open_type = "VIDEO";
    }
    else{
        std::cout << "错误：不支持的文件类型 (." << end_with << ")\n";
        raise(SIGINT);
    }
}

VirtualCamera::~VirtualCamera(void){
    this->close();
}

int VirtualCamera::open(void){
    if(this->open_type == "VIDEO"){
        try{
            cam.open(this->source_path);
        } catch(const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
        this->is_opened = cam.isOpened();
    }
    else{
        this->is_opened = true;
    }
    return (this->is_opened) ? 0 : -1;
}

int VirtualCamera::close(void){
    this->is_opened = false;
    this->cam.release();
    return 0;
}

cv::Mat VirtualCamera::get_frame(void){
    if(!this->is_opened){
        this->open();
    }

    cv::Mat frame;
    if(this->open_type == "VIDEO"){
        bool status = cam.read(frame);
        if(!status){
            std::cout << "视频结束" << '\n';
            raise(SIGINT);
            // cam.set(cv::CAP_PROP_POS_AVI_RATIO, 0);
            // status = cam.read(frame);
            // if(!status){
            //     std::cout << "重新读取失败" << '\n';
            //     raise(SIGINT);
            // }
        }
    }
    else{
        frame = cv::imread(this->source_path);
        if(frame.empty()){
            std::cout << "图片读取失败" << '\n';
            raise(SIGINT);
        }
    }

    return frame;
}