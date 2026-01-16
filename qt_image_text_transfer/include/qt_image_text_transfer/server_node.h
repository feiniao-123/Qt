#ifndef SERVER_NODE_H
#define SERVER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <memory>
#include <mutex>

namespace qt_image_text_transfer {

/**
 * @brief ROS2服务端节点类
 *
 * 负责：
 * 1. 发布图像消息到客户端
 * 2. 发布文字消息到客户端
 */
class ServerNode : public rclcpp::Node
{
public:
  ServerNode();
  ~ServerNode();

  // 发布图像
  void publishImage(const cv::Mat &image);
  // 发布文字
  void publishText(const std::string &text);

private:
  // 图像发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  // 文字发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;

  // 互斥锁保护发布操作
  std::mutex publish_mutex_;
};

} // namespace qt_image_text_transfer

#endif // SERVER_NODE_H
