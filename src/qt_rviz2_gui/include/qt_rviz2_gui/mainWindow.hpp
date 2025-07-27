/**
  ******************************************************************************
  * @file           : mainWindow.hpp
  * @author         : sun
  * @brief          : None
  * @attention      : None
  * @date           : 7/27/25
  ******************************************************************************
  */
#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include <QToolButton>
#include <QPushButton>
#include <QTimer>
#include <QLineEdit>
#include <QLabel>
#include <QCloseEvent>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
class QPushButton;
QT_END_NAMESPACE
namespace rviz_common {
class Display;
}

class MainWindow : public QMainWindow, public rviz_common::WindowManagerInterface {
    Q_OBJECT

public:
    MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent = nullptr);
    ~MainWindow() override;

    QWidget *getParentWindow() override;
    rviz_common::PanelDockWidget *addPane(const QString &name, QWidget *pane, Qt::DockWidgetArea area, bool floating) override;
    void setStatus(const QString &message) override;

protected:
    // void closeEvent(QCloseEvent *event) override;
    void showEvent(QShowEvent* event) override;   // 延迟初始化
    void closeEvent(QCloseEvent* event) override;

private slots:
    
    void onInitializeRViz();   // 真正初始化槽
private:
void sendJoystickCommand();              // Sends cmd_vel based on button input
    void updateFrame();                      // Slot to update the reference frame
    
    void setupGridDisplay();
    void setupTFDisplay();
  
    void setupRobotModelDisplay();
    bool eventFilter(QObject* obj, QEvent* event) override; // 声明事件过滤器

   

    QApplication *app_;
    QWidget *centralWidget_;
    QVBoxLayout *mainLayout_;
    QLineEdit *frameLineEdit_;               // Text box for the reference frame
    
    rviz_common::RenderPanel *renderPanel_=nullptr; // Render panel for RViz visualization
    rviz_common::Display *grid_;             // Grid display object
    rviz_common::Display *tf_display_;       // TF display object
   
    rviz_common::Display *robot_model_display_; // RobotModel display object
    rviz_common::VisualizationManager *manager_=nullptr;

    // ROS node and publisher for /cmd_vel
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
    
    

    
    
    
    
};

#endif // MAINWINDOW_HPP

