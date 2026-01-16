#include "qt_image_text_transfer/server_window.h"
#include "qt_image_text_transfer/server_node.h"
#include <QMessageBox>
#include <QDateTime>
#include <opencv2/core/utils/logger.hpp>
#include <utility>

namespace qt_image_text_transfer {

ServerWindow::ServerWindow(std::shared_ptr<ServerNode> node, QWidget *parent)
  : QMainWindow(parent)
  , node_(std::move(node))
  , is_running_(false)
  , source_type_(V4L2_CAMERA)
{
  setupUI();

  // 创建定时器，用于定时获取图像（30fps）
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &ServerWindow::onTimerTimeout);

  // 默认尝试打开/dev/video0
  source_combo_->setCurrentIndex(0);
  onSourceChanged(0);
}

ServerWindow::~ServerWindow()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
  if (timer_->isActive()) {
    timer_->stop();
  }
}

void ServerWindow::setupUI()
{
  setWindowTitle(QStringLiteral("服务端 - 图文传输系统"));
  resize(800, 600);

  // 创建中心部件
  auto *central_widget = new QWidget(this);
  setCentralWidget(central_widget);

  // 主布局
  auto *main_layout = new QVBoxLayout(central_widget);

  // ========== 图像源选择区域 ==========
  auto *source_group = new QGroupBox(QStringLiteral("图像源设置"), this);
  auto *source_layout = new QHBoxLayout(source_group);

  source_layout->addWidget(new QLabel(QStringLiteral("图像源:"), this));

  source_combo_ = new QComboBox(this);
  source_combo_->addItem(QStringLiteral("V4L2相机"));
  source_combo_->addItem(QStringLiteral("RTSP流"));
  source_layout->addWidget(source_combo_);

  url_edit_ = new QLineEdit(QStringLiteral("rtsp://localhost:8554/stream"), this);
  url_edit_->setEnabled(false);
  source_layout->addWidget(url_edit_);

  connect_btn_ = new QPushButton(QStringLiteral("连接"), this);
  source_layout->addWidget(connect_btn_);

  main_layout->addWidget(source_group);

  // ========== 图像预览区域 ==========
  auto *preview_group = new QGroupBox(QStringLiteral("图像预览"), this);
  auto *preview_layout = new QVBoxLayout(preview_group);

  preview_label_ = new QLabel(this);
  preview_label_->setMinimumSize(640, 480);
  preview_label_->setAlignment(Qt::AlignCenter);
  preview_label_->setStyleSheet(QStringLiteral("background-color: black; color: white;"));
  preview_label_->setText(QStringLiteral("未连接到图像源"));
  preview_layout->addWidget(preview_label_);

  main_layout->addWidget(preview_group);

  // ========== 文字发送区域 ==========
  auto *text_group = new QGroupBox(QStringLiteral("发送文字"), this);
  auto *text_layout = new QVBoxLayout(text_group);

  text_edit_ = new QTextEdit(this);
  text_edit_->setPlaceholderText(QStringLiteral("在此输入要发送到客户端的文字..."));
  text_layout->addWidget(text_edit_);

  send_btn_ = new QPushButton(QStringLiteral("发送文字"), this);
  text_layout->addWidget(send_btn_);

  main_layout->addWidget(text_group);

  // ========== 状态栏 ==========
  status_label_ = new QLabel(QStringLiteral("状态: 未连接"), this);
  main_layout->addWidget(status_label_);

  // ========== 连接信号槽 ==========
  connect(source_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ServerWindow::onSourceChanged);
  connect(connect_btn_, &QPushButton::clicked, this, [this]() {
    if (is_running_) {
      // 断开连接
      if (timer_->isActive()) {
        timer_->stop();
      }
      if (cap_.isOpened()) {
        cap_.release();
      }
      is_running_ = false;
      connect_btn_->setText(QStringLiteral("连接"));
      status_label_->setText(QStringLiteral("状态: 已断开"));
      preview_label_->clear();
      preview_label_->setText(QStringLiteral("未连接到图像源"));
    } else {
      // 连接
      if (source_type_ == V4L2_CAMERA) {
        QString device = QStringLiteral("/dev/video0");
        if (openV4L2Camera(device)) {
          is_running_ = true;
          connect_btn_->setText(QStringLiteral("断开"));
          status_label_->setText(QStringLiteral("状态: 已连接到V4L2相机"));
          timer_->start(33); // 约30fps
        }
      } else {
        QString url = url_edit_->text();
        if (openRTSPStream(url)) {
          is_running_ = true;
          connect_btn_->setText(QStringLiteral("断开"));
          status_label_->setText(QStringLiteral("状态: 已连接到RTSP流"));
          timer_->start(33); // 约30fps
        }
      }
    }
  });

  connect(send_btn_, &QPushButton::clicked, this, &ServerWindow::onSendText);
}

void ServerWindow::onSourceChanged(int index)
{
  if (is_running_) {
    QMessageBox::warning(this, QStringLiteral("警告"),
                         QStringLiteral("请先断开当前连接"));
    source_combo_->blockSignals(true);
    source_combo_->setCurrentIndex(source_type_ == V4L2_CAMERA ? 0 : 1);
    source_combo_->blockSignals(false);
    return;
  }

  if (index == 0) {
    source_type_ = V4L2_CAMERA;
    url_edit_->setEnabled(false);
  } else {
    source_type_ = RTSP_STREAM;
    url_edit_->setEnabled(true);
  }
}

bool ServerWindow::openV4L2Camera(const QString &device)
{
  if (cap_.isOpened()) {
    cap_.release();
  }

  cap_.open(device.toStdString());
  if (!cap_.isOpened()) {
    QMessageBox::critical(this, QStringLiteral("错误"),
                          QStringLiteral("无法打开V4L2设备: %1").arg(device));
    return false;
  }

  // 设置摄像头参数
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap_.set(cv::CAP_PROP_FPS, 30);

  return true;
}

bool ServerWindow::openRTSPStream(const QString &url)
{
  if (cap_.isOpened()) {
    cap_.release();
  }

  // 设置RTSP流的环境变量，降低延迟
  cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

  cap_.open(url.toStdString());
  if (!cap_.isOpened()) {
    QMessageBox::critical(this, QStringLiteral("错误"),
                          QStringLiteral("无法打开RTSP流: %1").arg(url));
    return false;
  }

  // 设置缓冲区大小以减少延迟
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

  return true;
}

void ServerWindow::onTimerTimeout()
{
  if (!cap_.isOpened() || !is_running_) {
    return;
  }

  cv::Mat frame;
  cap_ >> frame;

  if (frame.empty()) {
    status_label_->setText(QStringLiteral("状态: 无法读取帧"));
    return;
  }

  // 转换为QImage用于预览
  QImage qimage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
  qimage = qimage.rgbSwapped();

  // 更新预览
  updatePreview(qimage);

  // 发布到ROS
  node_->publishImage(frame);
}

void ServerWindow::updatePreview(const QImage &image)
{
  QPixmap pixmap = QPixmap::fromImage(image).scaled(
    preview_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  preview_label_->setPixmap(pixmap);
}

void ServerWindow::onSendText()
{
  QString text = text_edit_->toPlainText().trimmed();
  if (text.isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("警告"),
                         QStringLiteral("请输入要发送的文字"));
    return;
  }

  // 添加时间戳
  QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss"));
  QString message = QStringLiteral("[%1] %2").arg(timestamp, text);

  // 通过ROS发布
  node_->publishText(message.toStdString());

  // 清空输入框
  text_edit_->clear();

  status_label_->setText(QStringLiteral("状态: 文字已发送"));
}

} // namespace qt_image_text_transfer
