#include "qt_image_text_transfer/server_node.h"

namespace qt_image_text_transfer {

ServerNode::ServerNode()
  : Node("server_node")
{
  // 创建图像发布者
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "server/image", 10);

  // 创建文字发布者
  text_pub_ = this->create_publisher<std_msgs::msg::String>(
    "server/text", 10);

  RCLCPP_INFO(this->get_logger(), "服务端节点已启动");
}

ServerNode::~ServerNode()
{
  RCLCPP_INFO(this->get_logger(), "服务端节点已关闭");
}

void ServerNode::publishImage(const cv::Mat &image)
{
  std::lock_guard lock(publish_mutex_);

  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "尝试发布空图像");
    return;
  }

  try {
    // 将OpenCV Mat转换为ROS消息
    auto cv_image = std::make_shared<cv_bridge::CvImage>();
    cv_image->image = image;
    cv_image->encoding = sensor_msgs::image_encodings::BGR8;

    // 发布图像
    image_pub_->publish(*cv_image->toImageMsg());

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
  }
}

void ServerNode::publishText(const std::string &text)
{
  std::lock_guard lock(publish_mutex_);

  auto msg = std_msgs::msg::String();
  msg.data = text;
  text_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "发送文字: %s", text.c_str());
}

} // namespace qt_image_text_transfer
