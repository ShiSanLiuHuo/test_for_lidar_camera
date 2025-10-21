#include <cstdio>
#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("img_node") {
    RCLCPP_INFO(this->get_logger(), "Opening img...");

    // 创建 ROS 发布器
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_for_radar", 10);

    // 启动定时器发布图像
    image_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000),
                                std::bind(&CameraNode::PublishImage, this));

    PublishImage();
  }

private:
  void PublishImage() {
    if (count_ > max_count_) {
      count_ = 1;
    }
    std::string path = "/home/phoenix/ws_AImatch/img/img_test0" +
                       std::to_string(count_) + ".png";
    img = cv::imread(path);

    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open image!");
      return;
    }

    cv::resize(img, img, cv::Size(), 0.5, 0.5);

    // 转换颜色，BGRA->RGB
    cv::cvtColor(img, img, cv::COLOR_BGRA2RGB);

    // 转换为 ROS 图像消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    auto msg = cv_bridge::CvImage(header, "rgb8", img).toImageMsg();

    image_publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Published image: %s, size: %d * %d",
                path.c_str(), img.rows, img.cols);
    count_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  cv::VideoCapture cap_;
  cv::Mat img;

  int roi_x_ = 0;
  int roi_y_ = 0;
  int roi_width_ = 640;
  int roi_height_ = 640;

  int count_ = 1;
  const int max_count_ = 3;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
