#include <cstdio>
#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class WebMCameraNode : public rclcpp::Node {
public:
  WebMCameraNode() : Node("webm_camera_node") {
    // 声明参数，允许通过命令行或启动文件指定视频路径
    this->declare_parameter<std::string>("video_path", "");

    // 获取参数值
    std::string video_path = this->get_parameter("video_path").as_string();

    // 如果没有提供路径，使用默认值
    if (video_path.empty()) {
      video_path = "/home/phoenix/Videos/录屏/录屏 2025年09月16日 "
                   "19时44分24秒.webm"; // 默认WebM文件路径
      RCLCPP_WARN(this->get_logger(),
                  "No video path provided, using default: %s",
                  video_path.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Opening WebM video: %s",
                video_path.c_str());

    // 打开WebM视频文件
    cap_.open(video_path, cv::CAP_FFMPEG);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open WebM video!");
      return;
    }

    // 获取视频属性
    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "FPS unknown, defaulting to 30.");
      fps = 30.0;
    }
    int interval_ms = static_cast<int>(1000.0 / fps);

    // 跳转到 0 分 48 秒
    int start_time_sec = 48;
    int start_frame = static_cast<int>(fps * start_time_sec);
    cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame);

    // 获取视频总帧数和尺寸
    total_frames_ = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    frame_width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    frame_height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

    RCLCPP_INFO(this->get_logger(),
                "Video properties: %.1f FPS, %d frames, %dx%d", fps,
                total_frames_, frame_width_, frame_height_);

    // 读取第一帧
    if (!cap_.read(image_) || image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read initial frame.");
      return;
    }

    // 创建ROS发布器
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_for_radar", 10);

    // 启动发布定时器
    image_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(interval_ms),
                                std::bind(&WebMCameraNode::PublishImage, this));
  }

private:
  void PublishImage() {
    if (!cap_.read(image_) || image_.empty()) {
      RCLCPP_WARN(this->get_logger(), "WebM video ended. Restarting...");
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!cap_.read(image_) || image_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to restart WebM video.");
        return;
      }
    }

    // 提取ROI
    cv::Rect roi(roi_x_, roi_y_, roi_width_, roi_height_);

    // 确保ROI不超出图像边界
    roi.x = std::max(0, std::min(roi.x, image_.cols - 1));
    roi.y = std::max(0, std::min(roi.y, image_.rows - 1));
    roi.width = std::max(1, std::min(roi.width, image_.cols - roi.x));
    roi.height = std::max(1, std::min(roi.height, image_.rows - roi.y));

    cv::Mat roi_img = image_.clone();

    // 转换为RGB格式
    cv::cvtColor(roi_img, roi_img, cv::COLOR_BGR2RGB);

    cv::imshow("WebM Video", roi_img);
    cv::waitKey(1);

    // 创建并发布ROS消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    auto msg = cv_bridge::CvImage(header, "rgb8", roi_img).toImageMsg();
    image_publisher_->publish(*msg);

    RCLCPP_DEBUG(this->get_logger(), "Published WebM ROI image at (%d, %d)",
                 roi_x_, roi_y_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  cv::VideoCapture cap_;
  cv::Mat image_;

  int roi_x_;
  int roi_y_;
  int roi_width_;
  int roi_height_;

  int total_frames_;
  int frame_width_;
  int frame_height_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebMCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
