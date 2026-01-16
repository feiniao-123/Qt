# 基于 Qt 5 的服务端与客户端的图文传输系统

1. 技术栈
    - 操作系统: Ubuntu 22.04 LTS
    - 开发框架: ROS2 Humble
    - 编程语言: C++ (核心算法), Python 3.10.12(launch文件)
    - 图形界面: Qt 5.15.3
2. 功能：
    - 启动时，打开两个窗口，一个作为客户端，另一个作为服务端
    - 服务端通过 L4V2 相机或 RTSP 流媒体获取图像，然后实时发送到客户端，在客户端上显示
    - 在服务端输入文字，可以发送到客户端并在客户端显示

## 项目结构：

```
  qt_image_text_transfer/
  ├── CMakeLists.txt                                    # CMake构建配置
  ├── package.xml                                       # ROS2包配置
  ├── launch/
  │   └── image_text_transfer.launch.py                 # 启动文件
  ├── include/qt_image_text_transfer/
  │   ├── server_window.h                               # 服务端窗口头文件
  │   ├── client_window.h                               # 客户端窗口头文件
  │   ├── server_node.h                                 # 服务端节点头文件
  │   └── client_node.h                                 # 客户端节点头文件
  └── src/
      ├── main.cpp                                      # 主程序入口
      ├── server_window.cpp                             # 服务端窗口实现
      ├── client_window.cpp                             # 客户端窗口实现
      ├── server_node.cpp                               # 服务端节点实现
      └── client_node.cpp                               # 客户端节点实现
```

## 主要功能：

```
  ┌──────────────┬────────────────────────────────┐
  │     组件     │              功能              │
  ├──────────────┼────────────────────────────────┤
  │ ServerNode   │ 发布图像和文字消息到ROS话题    │
  ├──────────────┼────────────────────────────────┤
  │ ClientNode   │ 订阅图像和文字消息，并更新UI   │
  ├──────────────┼────────────────────────────────┤
  │ ServerWindow │ V4L2相机/RTSP流预览 + 文字发送 │
  ├──────────────┼────────────────────────────────┤
  │ ClientWindow │ 图像显示 + 接收文字显示        │
  └──────────────┴────────────────────────────────┘
```

## 构建命令：

```bash
cd /data
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

## 运行命令：

```bash
source install/setup.bash
ros2 launch qt_image_text_transfer image_text_transfer.launch.py
```