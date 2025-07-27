

# 项目名称

基于 RViz2 和 Qt 的机器人可视化与交互平台

---

## 项目简介

本项目基于 ROS 2 和 RViz 框架，结合 Qt 开发了一个机器人三维可视化与交互界面。
核心功能包括：

* 支持 RViz 默认视角控制器（Orbit视角）
* 实现基于鼠标的视角旋转、缩放和平移交互
* 动态加载机器人模型并实时显示 TF 变换
* 支持网格、坐标轴等基础场景元素显示
* 可通过 Qt 界面灵活配置坐标系与显示内容
## 🖼️ 界面预览
![运行演示](https://github.com/cheng9911/qt_rviz2_gui/raw/main/src/qt_rviz2_gui/image/output.gif)
---

## 主要功能

* **视角控制**：基于 `rviz_default_plugins/Orbit`，支持鼠标左键旋转、滚轮缩放、右键平移等操作。
* **交互工具**：集成 `MoveCamera` 工具，实现与视角控制的无缝交互。
* **机器人模型显示**：通过 `/robot_description` 话题加载 URDF 模型，实时渲染机械臂或机器人结构。
* **基础显示元素**：支持显示网格（Grid）、坐标轴（TF）等，便于环境感知。
* **Qt 集成**：基于 Qt Widgets，结合 `rviz_common::RenderPanel` 实现高性能渲染和事件管理。

---

## 环境要求

* ROS 2（Humble）
* rviz\_common 和 rviz\_default\_plugins（ROS 2 自带）
* Qt 5.15 及以上
* Ogre 3D（rviz 渲染引擎）

---
## 安装依赖
```bash
sudo apt-get update
sudo apt-get install qtbase5-private-dev libqt5svg5-dev libsdl-image1.2-dev libsdl1.2-dev -y
```
---
## 快速使用
```bash
# 克隆项目
mkdir qt_rviz2_ws && cd qt_rviz2_ws
mkdir src && cd src
git clone https://github.com/cheng9911/qt_rviz2_gui.git
cd .. && colcon build
source install/setup.bash
ros2 launch qt_rviz2_gui  display.launch.py
```
---

## 代码结构

* `src/`：主要源代码目录
* `include/`：头文件
* `launch/`：启动文件
* `README.md`：项目说明

---

## 未来计划

* 添加丰富的ui组件和交互功能
* 支持更多机器人
* 支持更多传感器数据可视化


---

## 联系方式

如有问题或建议，欢迎提交 Issue 或联系作者：

