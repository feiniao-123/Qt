#ifndef SERVER_WINDOW_H
#define SERVER_WINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QComboBox>
#include <memory>
#include <opencv2/opencv.hpp>

namespace qt_image_text_transfer {

// 前向声明
class ServerNode;

/**
 * @brief 服务端窗口类
 *
 * 功能：
 * 1. 显示从相机/RTSP获取的图像预览
 * 2. 发送文字到客户端
 * 3. 选择图像源（V4L2相机或RTSP流）
 */
class ServerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit ServerWindow(std::shared_ptr<ServerNode> node, QWidget *parent = nullptr);
  ~ServerWindow();

private slots:
  // 发送文字到客户端
  void onSendText();
  // 切换图像源
  void onSourceChanged(int index);
  // 定时器超时，获取并发布图像
  void onTimerTimeout();

private:
  void setupUI();
  bool openV4L2Camera(const QString &device);
  bool openRTSPStream(const QString &url);
  void updatePreview(const QImage &image);

  // ROS2节点
  std::shared_ptr<ServerNode> node_;

  // UI组件
  QLabel *preview_label_;      // 图像预览标签
  QComboBox *source_combo_;    // 图像源选择
  QLineEdit *url_edit_;        // RTSP URL输入
  QPushButton *connect_btn_;   // 连接按钮
  QTextEdit *text_edit_;       // 文字输入区域
  QPushButton *send_btn_;      // 发送按钮
  QLabel *status_label_;       // 状态标签

  // 定时器用于定时获取图像
  QTimer *timer_;

  // 视频捕获
  cv::VideoCapture cap_;
  bool is_running_;

  // 图像源类型
  enum SourceType {
    V4L2_CAMERA,
    RTSP_STREAM
  };
  SourceType source_type_;
};

} // namespace qt_image_text_transfer

#endif // SERVER_WINDOW_H
