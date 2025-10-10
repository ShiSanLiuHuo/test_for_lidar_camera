#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class ImageNode : public rclcpp::Node {
public:
  ImageNode() : Node("image_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/compressed_image", 10,
        std::bind(&ImageNode::callback, this, std::placeholders::_1));
    // 创建 ROS 发布器
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_for_radar", 10);
  }

private:
  void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
      auto total_start = std::chrono::high_resolution_clock::now();
      // msg->format 可能是 "jpeg" 或 "png"
      cv::Mat img = cv::imdecode(cv::Mat(msg->data),
                                 cv::IMREAD_COLOR); // 解码成 BGR cv::Mat
      if (!img.empty()) {
        // 转换颜色，BGR->RGB
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

        // 转换为 ROS 图像消息
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        auto msg = cv_bridge::CvImage(header, "rgb8", img).toImageMsg();

        image_publisher_->publish(*msg);
        //总时间
        auto total_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            total_end - total_start);
        RCLCPP_INFO(this->get_logger(), "total transformed time: %ld ms",
                    duration.count());
      }
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv::imdecode error: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
