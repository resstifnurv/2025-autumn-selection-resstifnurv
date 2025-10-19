#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "virtual_camera/VirtualCamera.hpp"
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

class CameraNode : public rclcpp::Node {

public:
    CameraNode(const rclcpp::NodeOptions &options);
    ~CameraNode() = default;
    void get_image(void);

private:
    std::string source_path;
    std::shared_ptr<VirtualCamera> cam;
    int fps;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};