#include <QApplication>
#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/executors.hpp>
#include "qt_image_text_transfer/server_window.h"
#include "qt_image_text_transfer/client_window.h"
#include "qt_image_text_transfer/server_node.h"
#include "qt_image_text_transfer/client_node.h"

/**
 * @brief 主函数
 *
 * 启动Qt应用，同时创建服务端和客户端窗口
 */
int main(int argc, char **argv)
{
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 初始化Qt应用
  QApplication app(argc, argv);

  // 创建服务端节点
  auto server_node = std::make_shared<qt_image_text_transfer::ServerNode>();

  // 创建客户端窗口（先创建窗口，因为节点需要窗口指针）
  auto client_window = std::make_unique<qt_image_text_transfer::ClientWindow>(nullptr);
  client_window->show();

  // 创建客户端节点，传入窗口指针
  auto client_node = std::make_shared<qt_image_text_transfer::ClientNode>(client_window.get());

  // 创建服务端窗口
  auto server_window = std::make_unique<qt_image_text_transfer::ServerWindow>(server_node);
  server_window->show();

  // 使用MultiThreadedExecutor同时spin两个节点
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(server_node);
  executor->add_node(client_node);

  // 标志位：控制ROS线程退出
  std::atomic<bool> running{true};

  // 在单独的线程中运行ROS2节点
  std::thread ros_thread([&]() {
    while (running && rclcpp::ok()) {
      executor->spin_once(std::chrono::milliseconds(10));
    }
  });

  // 运行Qt事件循环
  int result = app.exec();

  // 停止ROS线程
  running = false;

  // 等待ROS线程结束
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  // 清理
  rclcpp::shutdown();

  return result;
}
