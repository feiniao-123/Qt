#include "qt_image_text_transfer/client_window.h"
#include "qt_image_text_transfer/client_node.h"
#include <QDateTime>
#include <QScrollBar>

namespace qt_image_text_transfer {

ClientWindow::ClientWindow(std::shared_ptr<ClientNode> node, QWidget *parent)
  : QMainWindow(parent)
  , node_(node)
  , frame_count_(0)
  , message_count_(0)
{
  setupUI();
}

ClientWindow::~ClientWindow()
{
}

void ClientWindow::setupUI()
{
  setWindowTitle(QStringLiteral("客户端 - 图文传输系统"));
  resize(800, 600);

  // 创建中心部件
  auto *central_widget = new QWidget(this);
  setCentralWidget(central_widget);

  // 主布局
  auto *main_layout = new QVBoxLayout(central_widget);

  // ========== 图像显示区域 ==========
  auto *image_group = new QGroupBox(QStringLiteral("接收图像"), this);
  auto *image_layout = new QVBoxLayout(image_group);

  image_scroll_ = new QScrollArea(this);
  image_scroll_->setWidgetResizable(true);
  image_scroll_->setMinimumHeight(480);

  image_label_ = new QLabel(this);
  image_label_->setMinimumSize(640, 480);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setStyleSheet(QStringLiteral("background-color: black; color: white;"));
  image_label_->setText(QStringLiteral("等待接收图像..."));

  image_scroll_->setWidget(image_label_);
  image_layout->addWidget(image_scroll_);

  main_layout->addWidget(image_group);

  // ========== 文字显示区域 ==========
  auto *text_group = new QGroupBox(QStringLiteral("接收文字"), this);
  auto *text_layout = new QVBoxLayout(text_group);

  text_scroll_ = new QScrollArea(this);
  text_scroll_->setWidgetResizable(true);
  text_scroll_->setMinimumHeight(150);

  text_display_ = new QLabel(this);
  text_display_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  text_display_->setTextFormat(Qt::PlainText);
  text_display_->setWordWrap(true);
  text_display_->setText(QStringLiteral("等待接收文字...\n"));
  text_display_->setStyleSheet(QStringLiteral("background-color: #f0f0f0; padding: 10px;"));

  text_scroll_->setWidget(text_display_);
  text_layout->addWidget(text_scroll_);

  main_layout->addWidget(text_group);

  // ========== 状态栏 ==========
  status_label_ = new QLabel(
    QStringLiteral("状态: 运行中 | 接收帧数: 0 | 接收消息数: 0"), this);
  main_layout->addWidget(status_label_);
}

void ClientWindow::updateImage(const QImage &image)
{
  // 在标签中显示图像
  QPixmap pixmap = QPixmap::fromImage(image).scaled(
    image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  image_label_->setPixmap(pixmap);

  // 更新统计
  frame_count_++;
  status_label_->setText(
    QStringLiteral("状态: 运行中 | 接收帧数: %1 | 接收消息数: %2")
      .arg(frame_count_).arg(message_count_));
}

void ClientWindow::addTextMessage(const QString &message)
{
  // 获取当前时间
  QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss"));

  // 如果是第一条消息，清除默认文本
  if (message_count_ == 0) {
    text_display_->clear();
  }

  // 添加新消息
  QString current_text = text_display_->text();
  text_display_->setText(current_text + message + QStringLiteral("\n"));

  // 滚动到底部
  text_scroll_->verticalScrollBar()->setValue(
    text_scroll_->verticalScrollBar()->maximum());

  // 更新统计
  message_count_++;
  status_label_->setText(
    QStringLiteral("状态: 运行中 | 接收帧数: %1 | 接收消息数: %2")
      .arg(frame_count_).arg(message_count_));
}

} // namespace qt_image_text_transfer
