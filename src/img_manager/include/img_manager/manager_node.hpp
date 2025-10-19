#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <exception>
#include <opencv2/opencv.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

class ManagerNode : public rclcpp::Node {

public:
    ManagerNode(const rclcpp::NodeOptions &options);
    ~ManagerNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    cv::VideoWriter export_writer;
    void init_writer(std::string target_path);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    static cv::Mat mark_light_strip(cv::Mat image);
};