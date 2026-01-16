#ifndef CLIENT_NODE_H
#define CLIENT_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <functional>
#include <memory>

namespace qt_image_text_transfer {

// 前向声明
class ClientWindow;

/**
 * @brief ROS2客户端节点类
 *
 * 负责：
 * 1. 订阅服务端的图像消息
 * 2. 订阅服务端的文字消息
 * 3. 将接收到的数据传递给UI显示
 */
class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(ClientWindow *window);
  ~ClientNode();

private:
  // 图像接收回调
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  // 文字接收回调
  void textCallback(const std_msgs::msg::String::SharedPtr msg);

  // 图像订阅者
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // 文字订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;

  // UI窗口指针（用于显示接收到的数据）
  ClientWindow *window_;
};

} // namespace qt_image_text_transfer

#endif // CLIENT_NODE_H
