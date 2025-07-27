/**
 ******************************************************************************
 * @file           : mainWindow.cpp
 * @author         : sun
 * @brief          : None
 * @attention      : None
 * @date           : 7/27/25
 ******************************************************************************
 */
#include <OgreVector3.h>
#include "qt_rviz2_gui/mainWindow.hpp"
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

MainWindow::MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent)
    : QMainWindow(parent), app_(app), rviz_ros_node_(rviz_ros_node)
{
    setWindowTitle("ROS2 Qt RViz");
    renderPanel_ = new rviz_common::RenderPanel(this);
    mainLayout_ = new QVBoxLayout;
    centralWidget_ = new QWidget();

    // Add frame input box and button
    // QLabel *frameLabel = new QLabel("Reference Frame:");
    frameLineEdit_ = new QLineEdit("base");
    QPushButton *updateFrameButton = new QPushButton("Update Frame");
    connect(updateFrameButton, &QPushButton::clicked, this, &MainWindow::updateFrame);
    // mainLayout_->addWidget(frameLabel);
    mainLayout_->addWidget(frameLineEdit_);
    mainLayout_->addWidget(updateFrameButton);
    mainLayout_->addWidget(renderPanel_); // Add the render panel here

    centralWidget_->setLayout(mainLayout_);
    setCentralWidget(centralWidget_);
    centralWidget_->installEventFilter(this); // 关键安装代码
    // frameLineEdit_->setFocusPolicy(Qt::ClickFocus); // 仅点击时才拿焦点
    // updateFrameButton->setFocusPolicy(Qt::ClickFocus);
}
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == centralWidget_)
    { // 使用实际成员变量
        switch (event->type())
        {
        case QEvent::MouseMove:
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonRelease:
        case QEvent::Wheel:
            QApplication::sendEvent(renderPanel_, event);
            return true; // 拦截事件避免重复处理
        default:
            break;
        }
    }
    return QMainWindow::eventFilter(obj, event);
}
void MainWindow::onInitializeRViz()
{
    auto node = rviz_ros_node_.lock();
    if (!node)
    {
        qCritical() << "RosNodeAbstraction is invalid!";
        return;
    }

    renderPanel_->getRenderWindow()->initialize();
    auto clock = node->get_raw_node()->get_clock();
    manager_ = new rviz_common::VisualizationManager(renderPanel_, rviz_ros_node_,
                                                     this, clock);
    renderPanel_->initialize(manager_);
    renderPanel_->setMouseTracking(true);
    renderPanel_->raise();                                               // 置顶显示
    renderPanel_->setAttribute(Qt::WA_TransparentForMouseEvents, false); // 禁用穿透
    manager_->initialize();
    manager_->startUpdate();

    QTimer::singleShot(100, this, [this]
                       {
    // 通过窗口句柄激活OpenGL上下文
    if (auto glWindow = renderPanel_->findChild<QWindow*>()) {
        glWindow->requestActivate(); 
    } });
    QTimer::singleShot(50, this, [this]
                       {
        QString frame = frameLineEdit_->text();
        manager_->setFixedFrame(frame);
        manager_->getRootDisplayGroup()->setFixedFrame(frame);

        if (auto grid = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true)) {
            grid->subProp("Line Style")->setValue("Lines");
            grid->subProp("Color")->setValue(QColor(Qt::white));
        }

        if (auto tf = manager_->createDisplay("rviz_default_plugins/TF", "TF", true)) {
            tf->subProp("Show Axes")->setValue(true);
        }

        if (auto robot = manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel", true)) {
            robot->subProp("Description Topic")->setValue("/robot_description");
            robot->subProp("Visual Enabled")->setValue(true);
            
        }

        manager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
        QTimer::singleShot(0, this, [this] {
            renderPanel_->setFocus(Qt::MouseFocusReason);
            renderPanel_->setFocusPolicy(Qt::StrongFocus);
            renderPanel_->raise();
            renderPanel_->setMinimumSize(640, 480);
            // 添加这段代码
    auto toolManager = manager_->getToolManager();
    toolManager->setCurrentTool(toolManager->addTool("rviz_default_plugins/MoveCamera"));
            auto *orbit = manager_->getViewManager()->getCurrent();
            qDebug() << "Current view controller:"<< (orbit ? orbit->getClassId() : "nullptr");
            if (auto orbit = manager_->getViewManager()->getCurrent()) {
                orbit->subProp("Pitch")->setValue(-0.6); // 上下旋转
                orbit->subProp("Yaw")->setValue(1.57);   // 左右旋转
                orbit->subProp("Distance")->setValue(5.0); // 缩放距离
                Ogre::Vector3 ogre_vec(0, 0, 0);
                QVector3D qvec(ogre_vec.x, ogre_vec.y, ogre_vec.z);
                orbit->subProp("Focal Point")->setValue(qvec); // 旋转中心
            }
        }); });
    // 1. 让 RenderPanel 永远拥有焦点
    renderPanel_->setFocusPolicy(Qt::StrongFocus);
    renderPanel_->setFocus();

    // 2. 确保它不会被布局挤压到 0×0
    renderPanel_->setMinimumSize(640, 480);
}

// ---------- 析构 ----------
MainWindow::~MainWindow()
{
    // 让节点自然销毁，避免在 GUI 线程里 shutdown
    if (manager_)
    {
        manager_->stopUpdate();
        delete manager_;
        manager_ = nullptr;
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    event->accept();
}
void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    // 首次显示时再初始化，避免阻塞 GUI 线程
    QTimer::singleShot(0, this, &MainWindow::onInitializeRViz);
}

QWidget *MainWindow::getParentWindow()
{
    return this;
}

rviz_common::PanelDockWidget *MainWindow::addPane(const QString &name, QWidget *pane, Qt::DockWidgetArea area, bool floating)
{
    return nullptr;
}

void MainWindow::setStatus(const QString &message)
{
    // Optional: handle setting a status message here
}

void MainWindow::setupGridDisplay()
{
    QString frame_id = frameLineEdit_->text();

    // Initialize the grid display
    grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
    if (grid_)
    {
        grid_->subProp("Line Style")->setValue("Lines");
        grid_->subProp("Color")->setValue(QColor(Qt::white));
        grid_->subProp("Reference Frame")->setValue(frame_id);
        qDebug() << "Grid display configured for fixed frame:" << frame_id;
    }
    else
    {
        qDebug() << "Failed to create Grid display.";
    }
}

void MainWindow::setupTFDisplay()
{
    // Set up the TF display to show frames with a fixed frame
    tf_display_ = manager_->createDisplay("rviz_default_plugins/TF", "TF Display", true);
    if (tf_display_)
    {
        tf_display_->subProp("Show Axes")->setValue(true);
        qDebug() << "TF display configured with axes and names shown.";
    }
    else
    {
        qDebug() << "Failed to create TF display.";
    }
}

void MainWindow::setupRobotModelDisplay()
{
    // Set up the RobotModel display for the /robot_description topic
    robot_model_display_ = manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    if (robot_model_display_)
    {
        robot_model_display_->subProp("Description Topic")->setValue("/tb3_0/robot_description"); // Set the topic to /robot_description
        robot_model_display_->subProp("TF Prefix")->setValue("");                                 // Set TF prefix to empty if needed /tb3_0/robot_description
        qDebug() << "RobotModel display configured for /robot_description topic.";
    }
    else
    {
        qDebug() << "Failed to create RobotModel display.";
    }
}

void MainWindow::updateFrame()
{
    QString frame_id = frameLineEdit_->text();
    // // 添加线程安全检测
    // Update the grid display's reference frame
    if (grid_)
    {
        grid_->subProp("Reference Frame")->setValue(frame_id);
    }

    // Set the fixed frame in the FrameManager directly
    if (manager_ && manager_->getFrameManager())
    {
        manager_->setFixedFrame(frame_id);                        // Set for frame manager
        manager_->getRootDisplayGroup()->setFixedFrame(frame_id); // Set for root display group
        qDebug() << "FrameManager fixed frame updated to:" << frame_id;
    }
}
