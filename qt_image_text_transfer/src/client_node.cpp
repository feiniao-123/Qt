#include "qt_image_text_transfer/client_node.h"
#include "qt_image_text_transfer/client_window.h"
#include <QDateTime>

namespace qt_image_text_transfer {

ClientNode::ClientNode(ClientWindow *window)
  : Node("client_node"), window_(window)
{
  // 创建图像订阅者
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "server/image", 10,
    std::bind(&ClientNode::imageCallback, this, std::placeholders::_1));

  // 创建文字订阅者
  text_sub_ = this->create_subscription<std_msgs::msg::String>(
    "server/text", 10,
    std::bind(&ClientNode::textCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "客户端节点已启动");
}

ClientNode::~ClientNode()
{
  RCLCPP_INFO(this->get_logger(), "客户端节点已关闭");
}

void ClientNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // 将ROS消息转换为OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // 转换为QImage
    QImage qimage(
      cv_ptr->image.data,
      cv_ptr->image.cols,
      cv_ptr->image.rows,
      cv_ptr->image.step,
      QImage::Format_RGB888);

    // BGR转RGB
    qimage = qimage.rgbSwapped();

    // 在UI线程中更新图像
    QMetaObject::invokeMethod(window_, [this, qimage]() {
      window_->updateImage(qimage);
    }, Qt::QueuedConnection);

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
  }
}

void ClientNode::textCallback(const std_msgs::msg::String::SharedPtr msg)
{
  QString text = QString::fromStdString(msg->data);

  // 在UI线程中更新文字
  QMetaObject::invokeMethod(window_, [this, text]() {
    window_->addTextMessage(text);
  }, Qt::QueuedConnection);

  RCLCPP_INFO(this->get_logger(), "接收文字: %s", msg->data.c_str());
}

} // namespace qt_image_text_transfer
