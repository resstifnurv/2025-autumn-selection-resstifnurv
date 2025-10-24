#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "VirtualWriter.hpp"
#include "ImgProcUtil.hpp"
#include <exception>
#include <opencv2/opencv.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

class ManagerNode : public rclcpp::Node {

public:
    ManagerNode(const rclcpp::NodeOptions &options);
    ~ManagerNode() = default;

private:
    std::string target_path;
    int fps;
    int width, height;
    std::shared_ptr<VirtualWriter> writer;
    std::string arg, export_mode;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};