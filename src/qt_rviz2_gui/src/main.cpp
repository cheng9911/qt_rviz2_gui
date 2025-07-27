#include <QApplication>
#include <memory>
#include "qt_rviz2_gui/mainWindow.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
 std::cout << "RViz Render Node started." << std::endl;
    auto ros_node_abs = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
 std::cout << "RViz Render Node started." << std::endl;
    auto mainWindow = std::make_shared<MainWindow>(&app, ros_node_abs);
     std::cout << "RViz Render Node started." << std::endl;
    mainWindow->show();
    std::cout << "RViz Render Node started." << std::endl;
    while (rclcpp::ok()) {
        app.processEvents();
    }

    return 0;
}


