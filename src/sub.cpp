#include <iostream>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageDisplayNode : public rclcpp::Node {
public:
  ImageDisplayNode() : Node("image_display_node") {
    using std::placeholders::_1;
    // 订阅 /image_for_radar 图像话题
    raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_for_radar", rclcpp::QoS(10).best_effort(),
        std::bind(&ImageDisplayNode::raw_callback, this, _1));

    // 订阅 /detect 图像话题
    detect_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/detector/img_detect", rclcpp::QoS(10).best_effort(),
        std::bind(&ImageDisplayNode::detect_callback, this, _1));

    // 订阅 /img_car 图像话题
    img_car_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/detector/img_car", rclcpp::QoS(10).best_effort(),
        std::bind(&ImageDisplayNode::img_car_callback, this, _1));

    // 订阅 /img_armor 图像话题
    img_armor_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/detector/img_armor", rclcpp::QoS(10).best_effort(),
        std::bind(&ImageDisplayNode::img_armor_callback, this, _1));

    // 订阅 /match_draw 图像话题
    img_match_draw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/match_draw", rclcpp::QoS(10).best_effort(),
        std::bind(&ImageDisplayNode::img_match_draw_callback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to /detector/img_detect and /detector/img_car");
  }

private:
  void detect_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    show_image(msg, "Detect Image", 0.5);
  }

  void img_car_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    show_image(msg, "Img Car Image", 0.25);
  }

  void img_armor_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    show_image(msg, "Img Armor Image", 0.25);
  }

  void show_image(const sensor_msgs::msg::Image::SharedPtr msg,
                  const std::string &window_name, double scale_) {
    try {
      // 自动选择编码格式，如果失败默认用 bgr8
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      } catch (...) {
        cv_ptr = cv_bridge::toCvCopy(msg); // fallback
      }

      cv::Mat resized_image;
      double scale = scale_;
      cv::resize(cv_ptr->image, resized_image, cv::Size(), scale, scale);
      cv::imshow(window_name, resized_image);

      // 如果任意窗口按下 'q' 键退出
      if (cv::waitKey(1) == 'q') {
        rclcpp::shutdown();
      }

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr detect_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_car_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_armor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_match_draw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;

  void raw_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    show_image(msg, "Raw Image", 0.5);
  }

  void img_match_draw_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    show_image(msg, "Img Match Draw Image", 0.25);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageDisplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
