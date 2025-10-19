#include "virtual_camera/camera_node.hpp"
#include "virtual_camera/VirtualCamera.hpp"
#include <memory>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <rclcpp/node_options.hpp>
#include <cv_bridge/cv_bridge.h>

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("virtual_camera", options){
    this->fps = this->declare_parameter("fps", 30);
    this->source_path = this->declare_parameter("source_path", "video.mp4");

    RCLCPP_INFO(this->get_logger(), "正在初始化相机...");
    this->cam = std::make_shared<VirtualCamera>(this->source_path);
    if(cam->open() != 0){
        RCLCPP_ERROR(this->get_logger(), "视频读取失败");
        return ;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "视频读取成功");
        RCLCPP_INFO(this->get_logger(), "帧率：%d", this->fps);
        RCLCPP_INFO(this->get_logger(), "视频路径：%s", this->source_path.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "相机初始化成功...");

    RCLCPP_INFO(this->get_logger(), "正在初始化图像发布器...");
    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    RCLCPP_INFO(this->get_logger(), "图像发布器初始化成功...");

    RCLCPP_INFO(this->get_logger(), "正在初始化计时器...");
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / fps),
        std::bind(&CameraNode::get_image, this)
    );
    RCLCPP_INFO(this->get_logger(), "计时器初始化成功...");
}

void CameraNode::get_image(void){
    cv::Mat image = this->cam->get_frame();
    rclcpp::Time stamp = this->now();

    std_msgs::msg::Header header;
    header.set__stamp(stamp);
    header.set__frame_id("camera_frame");

    auto img_msg = cv_bridge::CvImage(
        header, "bgr8", image
    ).toImageMsg();

    image_pub_->publish(*img_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)