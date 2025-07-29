// #include "mainwindow.h"
// #include <QApplication>
// #include <rclcpp/rclcpp.hpp>

// int main(int argc, char *argv[]) {
//     QApplication a(argc, argv);
//     MainWindow w;
//      // qDebug() << "Hello from Qt!444";
//     w.show();
//     // qDebug() << "Hello from Qt!444";
//     return a.exec();
// }
#include <QApplication>
#include <memory>
#include "mainwindow.h"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include <QLoggingCategory>
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    QLoggingCategory::setFilterRules("qt.qpa.input=true");   // 打开输入事件日
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
