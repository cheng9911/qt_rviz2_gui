#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QListWidget>
#include <QMessageBox>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include <QToolButton>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QCloseEvent>
#include <QDebug>

#include <OgreVector3.h>
#include <QShowEvent>
#include <QGridLayout>
#include <QTimer>
#include <QCloseEvent>
#include <QVBoxLayout>
#include <geometry_msgs/msg/point.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_window.hpp>
#include <QVector3D>
#include <QDebug>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
class QPushButton;
QT_END_NAMESPACE
namespace rviz_common {
class Display;
}
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow,public rviz_common::WindowManagerInterface  {
    Q_OBJECT

public:
    explicit MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,QWidget *parent = nullptr);
    ~MainWindow();

    QWidget *getParentWindow() override;
    rviz_common::PanelDockWidget *addPane(const QString &name, QWidget *pane, Qt::DockWidgetArea area, bool floating) override;
    void setStatus(const QString &message) override;

protected:
    // void closeEvent(QCloseEvent *event) override;
    void showEvent(QShowEvent* event) override;   // 延迟初始化
    void closeEvent(QCloseEvent* event) override;

private slots:

    void onTimerTimeout();
    void on_pushButton_1_pressed();

    void on_pushButton_1_released();

    void on_pushButton_2_pressed();

    void on_pushButton_2_released();

    void on_pushButton_3_pressed();

    void on_pushButton_3_released();

    void on_pushButton_4_pressed();

    void on_pushButton_4_released();

    void on_pushButton_5_pressed();

    void on_pushButton_5_released();

    void on_pushButton_6_pressed();

    void on_pushButton_6_released();

    void on_pushButton_7_pressed();

    void on_pushButton_7_released();

    void on_pushButton_8_pressed();

    void on_pushButton_8_released();

    void on_pushButton_9_pressed();

    void on_pushButton_9_released();

    void on_pushButton_10_pressed();

    void on_pushButton_10_released();

    void on_pushButton_11_pressed();

    void on_pushButton_11_released();

    void on_pushButton_12_pressed();

    void on_pushButton_12_released();

    void on_pushButton_13_pressed();

    void on_pushButton_13_released();

    void on_pushButton_14_pressed();

    void on_pushButton_14_released();

    void on_pushButton_clicked();

    void on_pushButton_15_pressed();

    void on_pushButton_15_released();

    void on_pushButton_16_pressed();

    void on_pushButton_16_released();

    void on_pushButton_17_pressed();

    void on_pushButton_17_released();

    void on_pushButton_18_pressed();

    void on_pushButton_18_released();

    void on_pushButton_19_pressed();

    void on_pushButton_19_released();

    void on_pushButton_20_pressed();

    void on_pushButton_20_released();


    void on_pushButton_2_1_pressed();

    void on_pushButton_2_1_released();

    void on_pushButton_2_2_pressed();

    void on_pushButton_2_2_released();

    void on_pushButton_2_3_pressed();

    void on_pushButton_2_3_released();

    void on_pushButton_2_4_pressed();

    void on_pushButton_2_4_released();

    void on_pushButton_2_5_pressed();

    void on_pushButton_2_5_released();

    void on_pushButton_2_6_pressed();

    void on_pushButton_2_6_released();

    void on_pushButton_2_7_pressed();

    void on_pushButton_2_7_released();

    void on_pushButton_2_8_pressed();

    void on_pushButton_2_8_released();

    void on_pushButton_2_9_pressed();

    void on_pushButton_2_9_released();

    void on_pushButton_2_10_pressed();

    void on_pushButton_2_10_released();

    void on_pushButton_2_11_pressed();

    void on_pushButton_2_11_released();

    void on_pushButton_2_12_pressed();

    void on_pushButton_2_12_released();

    void on_pushButton_2_13_pressed();

    void on_pushButton_2_13_released();

    void on_pushButton_2_14_pressed();

    void on_pushButton_2_14_released();

    void on_pushButton_2_15_pressed();

    void on_pushButton_2_15_released();

    void on_pushButton_2_16_pressed();

    void on_pushButton_2_16_released();

    void on_pushButton_2_17_pressed();

    void on_pushButton_2_17_released();

    void on_pushButton_2_18_pressed();

    void on_pushButton_2_18_released();

    void on_pushButton_2_19_pressed();

    void on_pushButton_2_19_released();

    void on_pushButton_2_20_pressed();

    void on_pushButton_2_20_released();

    void onInitializeRViz();   // 真正初始化槽
private:
    void sendJoystickCommand();              // Sends cmd_vel based on button input
    void updateFrame();                      // Slot to update the reference frame

    void setupGridDisplay();
    void setupTFDisplay();

    void setupRobotModelDisplay();
    // bool eventFilter(QObject* obj, QEvent* event) override; // 声明事件过滤器


    Ui::MainWindow *ui;
    QApplication *app_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    QTimer *publishTimer_;
    size_t count_;
    rocos::Robot *robot_ptr0 = nullptr;
    rocos::Robot *robot_ptr1 = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    void updateJointPosition();
    QTimer *m_timer;       // 共用的定时器
    int m_activeButton;
    void initROS();

    bool isEnabled = true;
    rocos::HardwareInterface *hw;

    rocos::HardwareInterface *hw1;

    rviz_common::RenderPanel *renderPanel_=nullptr; // Render panel for RViz visualization
    rviz_common::Display *grid_;             // Grid display object
    rviz_common::Display *tf_display_;       // TF display object

    rviz_common::Display *robot_model_display_; // RobotModel display object
    rviz_common::VisualizationManager *manager_=nullptr;

    // ROS node and publisher for /cmd_vel
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

#endif // MAINWINDOW_H
