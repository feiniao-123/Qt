"""
图文传输系统启动文件

启动服务端和客户端窗口，用于图像和文字传输
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成launch描述
    """
    return LaunchDescription([
        # 启动图文传输程序（包含服务端和客户端）
        # 不使用name参数，让代码中的节点名称生效
        Node(
            package='qt_image_text_transfer',
            executable='qt_image_text_transfer',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])
