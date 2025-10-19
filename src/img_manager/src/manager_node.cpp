#include <cv_bridge/cv_bridge.h>
#include <img_manager/manager_node.hpp>

cv::Mat ManagerNode::mark_light_strip(cv::Mat image){
    return image;
}

void ManagerNode::init_writer(std::string target_path){
    this->export_writer.open(
        target_path,
        cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
        30,
        cv::Size(1280, 1024)
    );
    if (!export_writer.isOpened()) {
        std::cerr << "Error: Could not open the VideoWriter!" << std::endl;
    }
}

void ManagerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    try {
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(
            msg,
            sensor_msgs::image_encodings::BGR8
        );
        auto img = img_ptr->image;
        auto img_marked = mark_light_strip(img);
        this->export_writer.write(img_marked);
        // cv::waitKey(10);
    }
    catch(cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    catch(std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Caught Exception: %s", e.what());
        raise(SIGABRT);
    }
}

ManagerNode::ManagerNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("manager_node", options){
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s", this->get_name());
    this->init_writer("resources/output/output.avi");
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { this->image_callback(msg); }
    );
}

ManagerNode::~ManagerNode(){
    this->export_writer.release();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ManagerNode)