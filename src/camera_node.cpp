#include <cstdio>
#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    RCLCPP_INFO(this->get_logger(), "Opening video...");

    cap_.open("/home/phoenix/Desktop/1.mp4",
              cv::CAP_FFMPEG); //  /home/phoenix/zk/save_stuff/20.mp4  or
    //  /home/phoenix/Desktop/1.mp4
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video!");
      return;
    }

    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "FPS unknown, defaulting to 30.");
      fps = 30.0;
    }
    int interval_ms = static_cast<int>(1000.0 / fps);

    // 跳转到 7 分 08 秒
    int start_time_sec = 7 * 60 + 8;
    int start_frame = static_cast<int>(fps * start_time_sec);
    cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame);

    // 读取一帧图像
    if (!cap_.read(image_) || image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read initial frame.");
      return;
    }

    // 根据图像大小动态计算ROI大小（最大640）
    roi_width_ = std::min(640, image_.cols);
    roi_height_ = std::min(640, image_.rows);

    // 计算滑块最大值，确保ROI不会越界
    int max_x = std::max(0, image_.cols - roi_width_);
    int max_y = std::max(0, image_.rows - roi_height_);

    // ROI 默认居中
    roi_x_ = max_x / 2;
    roi_y_ = max_y / 2;

    // 创建滑块窗口
    cv::namedWindow("Adjust ROI", cv::WINDOW_NORMAL);
    cv::resizeWindow("Adjust ROI", 1600, 1200);
    cv::createTrackbar("ROI X", "Adjust ROI", &roi_x_, max_x);
    cv::createTrackbar("ROI Y", "Adjust ROI", &roi_y_, max_y);

    RCLCPP_INFO(this->get_logger(),
                "Adjust ROI using sliders, press 'q' to confirm.");

    // 调整界面循环
    while (rclcpp::ok()) {
      cv::Mat preview = image_.clone();
      cv::Rect roi_rect(roi_x_, roi_y_, roi_width_, roi_height_);
      cv::rectangle(preview, roi_rect, cv::Scalar(0, 255, 0), 2);
      cv::imshow("Adjust ROI", preview);

      char key = static_cast<char>(cv::waitKey(30));
      if (key == 'q' || key == 'Q') {
        break;
      }
    }

    cv::destroyWindow("Adjust ROI");
    cv::waitKey(1);

    RCLCPP_INFO(this->get_logger(), "ROI confirmed at (%d, %d), size (%d x %d)",
                roi_x_, roi_y_, roi_width_, roi_height_);

    // 创建 ROS 发布器
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_for_radar", 10);

    // 启动定时器发布图像
    image_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(interval_ms),
                                std::bind(&CameraNode::PublishImage, this));

    PublishImage();
  }

private:
  void PublishImage() {
    if (!cap_.read(image_) || image_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Video ended. Restarting...");
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!cap_.read(image_) || image_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to restart video.");
        return;
      }
    }

    // 转换颜色，BGR->RGB
    cv::cvtColor(image_, image_, cv::COLOR_BGR2RGB);

    cv::imshow("Published Image", image_);
    cv::waitKey(1);

    // 转换为 ROS 图像消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    auto msg = cv_bridge::CvImage(header, "rgb8", image_).toImageMsg();

    image_publisher_->publish(*msg);

    RCLCPP_INFO(this->get_logger(), "Published image at (%d, %d)", roi_x_,
                roi_y_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  cv::VideoCapture cap_;
  cv::Mat image_;

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
