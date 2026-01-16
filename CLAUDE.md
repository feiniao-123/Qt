# CLAUDE.md

## 代码风格和方式
- 主要语言：C++
- 框架：ROS2 Humble
- 图形界面：Qt5.15.3
- **Error Handling**: Use custom Error classes and try-catch blocks in async actions.
- 缩进习惯：K&R风格
- 注释习惯：使用中文进行适当的注释
- 代码运行环境：Ubuntu 22.04 LTS

## 构建

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```