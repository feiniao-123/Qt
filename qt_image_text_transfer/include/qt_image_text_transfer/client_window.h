#ifndef CLIENT_WINDOW_H
#define CLIENT_WINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QDateTime>
#include <memory>

namespace qt_image_text_transfer {

// 前向声明
class ClientNode;

/**
 * @brief 客户端窗口类
 *
 * 功能：
 * 1. 接收并显示服务端发送的图像
 * 2. 接收并显示服务端发送的文字
 */
class ClientWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit ClientWindow(std::shared_ptr<ClientNode> node, QWidget *parent = nullptr);
  ~ClientWindow();

  // 更新接收到的图像
  void updateImage(const QImage &image);
  // 添加接收到的文字消息
  void addTextMessage(const QString &message);

private:
  void setupUI();

  // ROS2节点
  std::shared_ptr<ClientNode> node_;

  // UI组件
  QLabel *image_label_;        // 显示接收的图像
  QLabel *text_display_;       // 显示接收的文字
  QScrollArea *image_scroll_;  // 图像滚动区域
  QScrollArea *text_scroll_;   // 文字滚动区域
  QLabel *status_label_;       // 状态标签

  // 统计信息
  int frame_count_;
  int message_count_;
};

} // namespace qt_image_text_transfer

#endif // CLIENT_WINDOW_H
