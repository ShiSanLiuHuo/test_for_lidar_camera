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

    img = cv::imread("/home/phoenix/ws_AImatch/img/img_test01.png");

    //  /home/phoenix/Desktop/1.mp4
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open image!");
      return;
    }

    // 创建 ROS 发布器
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_for_radar", 10);

    // 启动定时器发布图像
    image_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&CameraNode::PublishImage, this));

    PublishImage();
  }

private:
  void PublishImage() {
    image_ = img.clone();
    // 转换颜色，BGRA->RGB
    cv::cvtColor(image_, image_, cv::COLOR_BGRA2RGB);

    cv::imshow("Published Image", image_);
    cv::waitKey(1);

    // 转换为 ROS 图像消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    auto msg = cv_bridge::CvImage(header, "rgb8", image_).toImageMsg();

    image_publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  cv::VideoCapture cap_;
  cv::Mat image_;
  cv::Mat img;

  int roi_x_ = 0;
  int roi_y_ = 0;
  int roi_width_ = 640;
  int roi_height_ = 640;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
