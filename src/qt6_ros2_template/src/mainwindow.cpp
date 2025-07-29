#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent)
    : QMainWindow(parent), app_(app), rviz_ros_node_(rviz_ros_node),ui(new Ui::MainWindow)
    {
    ui->setupUi(this);
    rocos::HardwareInterface *hw;

    rocos::HardwareInterface *hw1;
    initROS();
    node_->declare_parameter("sim", true);
    node_->declare_parameter("urdf_model_path", "");
    node_->declare_parameter("base", "base_link");
    node_->declare_parameter("tip", "link_7");
    bool sim_feature = node_->get_parameter("sim").as_bool();
    std::string urdf_path = node_->get_parameter("urdf_model_path").as_string();
    std::string base = node_->get_parameter("base").as_string();
    std::string tip = node_->get_parameter("tip").as_string();
    if(sim_feature)
    {
        hw = new rocos::HardwareSim(20); // 仿真
        hw1 = new rocos::HardwareSim(20); // 仿真
    }
    else{
    hw = new rocos::Hardware(urdf_path, 0); // 真实机械臂
    hw1 = new rocos::Hardware(urdf_path, 1); // 真实机械臂
    }
   
    robot_ptr0 =new  rocos::Robot(hw, urdf_path, base, tip);
    robot_ptr1 =new  rocos::Robot(hw1, urdf_path, base, tip);
    

    m_timer = new QTimer(this);
    m_timer->setInterval(100);  // 100ms间隔
    connect(m_timer, &QTimer::timeout, this, &MainWindow::onTimerTimeout);
 
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateJointPosition);
    timer->start(100); // 100ms刷新一次

    ui->pushButton->setText("使能");

    renderPanel_ = new rviz_common::RenderPanel(ui->renderPanelPlaceholder);
    auto layout = new QVBoxLayout(ui->renderPanelPlaceholder);
    ui->frameLineEdit->setText("base_link");
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(renderPanel_);
    connect(ui->updateFrameButton, &QPushButton::clicked, this, &MainWindow::updateFrame);

}




void MainWindow::initROS() {
    // 初始化ROS 2节点
    // rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("joint_angle_publisher");

    // 创建关节状态发布者，话题名为"joint_states"，队列大小为10
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}


// 只显示最新的关节位置（无时间戳）
void MainWindow::updateJointPosition()
{
    double joint1 = robot_ptr0->getJointPosition(0);
    double joint2 = robot_ptr0->getJointPosition(1);
    double joint3 = robot_ptr0->getJointPosition(2);
    double joint4 = robot_ptr0->getJointPosition(3);
    double joint5 = robot_ptr0->getJointPosition(4);
    double joint6 = robot_ptr0->getJointPosition(5);
    double joint7 = robot_ptr0->getJointPosition(6);
    double x_1 = robot_ptr0->getFlange().p.x();
    double y_1 = robot_ptr0->getFlange().p.y();
    double z_1 = robot_ptr0->getFlange().p.z();

    double joint2_1 = robot_ptr1->getJointPosition(0);
    double joint2_2 = robot_ptr1->getJointPosition(1);
    double joint2_3 = robot_ptr1->getJointPosition(2);
    double joint2_4 = robot_ptr1->getJointPosition(3);
    double joint2_5 = robot_ptr1->getJointPosition(4);
    double joint2_6 = robot_ptr1->getJointPosition(5);
    double joint2_7 = robot_ptr1->getJointPosition(6);
    double x_2 = robot_ptr1->getFlange().p.x();
    double y_2 = robot_ptr1->getFlange().p.y();
    double z_2 = robot_ptr1->getFlange().p.z();
    // double joint8 = robot_ptr0->getJointPosition(7);
    // 检查ROS 2是否仍在运行
    if (!rclcpp::ok()) return;

    // 创建关节状态消息
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"};
    joint_state.position = {joint1,joint2,joint3,joint4,joint5,joint6,joint7};
    // 设置时间戳
    joint_state.header.stamp = node_->get_clock()->now();
    joint_pub_->publish(joint_state);
    // 直接设置文本，只显示关节角度值
    ui->textEdit->setText(QString("关节1: %1").arg(joint1, 0, 'f', 4));
    ui->textEdit_2->setText(QString("关节2: %1").arg(joint2, 0, 'f', 4));
    ui->textEdit_3->setText(QString("关节3: %1").arg(joint3, 0, 'f', 4));
    ui->textEdit_4->setText(QString("关节4: %1").arg(joint4, 0, 'f', 4));
    ui->textEdit_5->setText(QString("关节5: %1").arg(joint5, 0, 'f', 4));
    ui->textEdit_6->setText(QString("关节6: %1").arg(joint6, 0, 'f', 4));
    ui->textEdit_7->setText(QString("关节7: %1").arg(joint7, 0, 'f', 4));
    ui->textEdit_x1->setText(QString("X: %1").arg(x_1, 0, 'f', 4));
    ui->textEdit_y1->setText(QString("Y: %1").arg(y_1, 0, 'f', 4));
    ui->textEdit_z1->setText(QString("Z: %1").arg(z_1, 0, 'f', 4));

    ui->textEdit2->setText(QString("关节1: %1").arg(joint2_1, 0, 'f', 4));
    ui->textEdit2_2->setText(QString("关节2: %1").arg(joint2_2, 0, 'f', 4));
    ui->textEdit2_3->setText(QString("关节3: %1").arg(joint2_3, 0, 'f', 4));
    ui->textEdit2_4->setText(QString("关节4: %1").arg(joint2_4, 0, 'f', 4));
    ui->textEdit2_5->setText(QString("关节5: %1").arg(joint2_5, 0, 'f', 4));
    ui->textEdit2_6->setText(QString("关节6: %1").arg(joint2_6, 0, 'f', 4));
    ui->textEdit2_7->setText(QString("关节7: %1").arg(joint2_7, 0, 'f', 4));
    ui->textEdit_x2->setText(QString("X: %1").arg(x_2, 0, 'f', 4));
    ui->textEdit_y2->setText(QString("Y: %1").arg(y_2, 0, 'f', 4));
    ui->textEdit_z2->setText(QString("Z: %1").arg(z_2, 0, 'f', 4));
}

void MainWindow::on_pushButton_1_pressed()
{
    m_activeButton = 1;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}

void MainWindow::onTimerTimeout()
{
    switch (m_activeButton) {
    case 1:
        // 按钮1的持续操作
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J0,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 2:
        // 按钮2的持续操作（示例：不同参数）
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J0,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 3:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J1,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 4:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J1,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 5:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J2,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 6:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J2,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 7:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J3,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 8:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J3,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 9:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J4,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 10:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J4,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 11:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J5,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 12:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J5,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 13:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J6,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 14:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::J6,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 15:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_X,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 16:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_X,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 17:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Y,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 18:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Y,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 19:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Z,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 20:
        robot_ptr0->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Z,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 21:
        // 按钮1的持续操作
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J0,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 22:
        // 按钮2的持续操作（示例：不同参数）
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J0,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 23:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J1,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 24:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J1,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 25:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J2,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 26:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J2,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 27:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J3,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 28:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J3,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 29:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J4,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 30:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J4,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 31:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J5,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 32:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J5,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 33:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J6,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 34:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::J6,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 35:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_X,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 36:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_X,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 37:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Y,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 38:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Y,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    case 39:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Z,
                             rocos::Robot::DRAGGING_DIRRECTION::NEGATIVE,
                             0.1, 0.1);
        break;
    case 40:
        robot_ptr1->Dragging(rocos::Robot::DRAGGING_FLAG::BASE_Z,
                             rocos::Robot::DRAGGING_DIRRECTION::POSITION,
                             0.1, 0.1);
        break;
    default:
        break;
    }
}



void MainWindow::on_pushButton_1_released()
{
    if (m_activeButton == 1) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_2_pressed()
{
    m_activeButton = 2;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_released()
{
    if (m_activeButton == 2) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_3_pressed()
{
    m_activeButton = 3;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_3_released()
{
    if (m_activeButton == 3) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_4_pressed()
{
    m_activeButton = 4;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_4_released()
{
    if (m_activeButton == 4) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_5_pressed()
{
    m_activeButton = 5;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_5_released()
{
    if (m_activeButton == 5) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_6_pressed()
{
    m_activeButton = 6;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_6_released()
{
    if (m_activeButton == 6) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_7_pressed()
{
    m_activeButton = 7;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_7_released()
{
    if (m_activeButton == 7) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_8_pressed()
{
    m_activeButton = 8;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_8_released()
{
    if (m_activeButton == 8) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_9_pressed()
{
    m_activeButton = 9;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_9_released()
{
    if (m_activeButton == 9) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_10_pressed()
{
    m_activeButton = 10;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_10_released()
{
    if (m_activeButton == 10) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_11_pressed()
{
    m_activeButton = 11;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_11_released()
{
    if (m_activeButton == 11) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_12_pressed()
{
    m_activeButton = 12;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_12_released()
{
    if (m_activeButton == 12) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_13_pressed()
{
    m_activeButton = 13;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_13_released()
{
    if (m_activeButton == 13) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_14_pressed()
{
    m_activeButton = 14;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_14_released()
{
    if (m_activeButton == 14) {
        m_activeButton = 0;
        m_timer->stop();
    }
}


void MainWindow::on_pushButton_clicked()
{
    isEnabled = !isEnabled;

    // 根据状态更新按钮文本
    if (isEnabled) {
        ui->pushButton->setText("使能");
        robot_ptr0->setDisabled();
        robot_ptr1->setDisabled();
    } else {
        ui->pushButton->setText("下使能");
        robot_ptr0->setEnabled();
        robot_ptr1->setEnabled();
    }
}


void MainWindow::on_pushButton_15_pressed()
{
    m_activeButton = 15;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_15_released()
{
    if (m_activeButton == 15) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_16_pressed()
{
    m_activeButton = 16;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_16_released()
{
    if (m_activeButton == 16) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_17_pressed()
{
    m_activeButton = 17;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_17_released()
{
    if (m_activeButton == 17) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_18_pressed()
{
    m_activeButton = 18;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_18_released()
{
    if (m_activeButton == 18) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_19_pressed()
{
    m_activeButton = 19;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_19_released()
{
    if (m_activeButton == 19) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_20_pressed()
{
    m_activeButton = 20;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_20_released()
{
    if (m_activeButton == 20) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_1_pressed()
{
    m_activeButton = 21;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_1_released()
{
    if (m_activeButton == 21) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_2_pressed()
{
    m_activeButton = 22;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_2_released()
{
    if (m_activeButton == 22) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_3_pressed()
{
    m_activeButton = 23;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_3_released()
{
    if (m_activeButton == 23) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_4_pressed()
{
    m_activeButton = 24;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_4_released()
{
    if (m_activeButton == 24) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_5_pressed()
{
    m_activeButton = 25;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_5_released()
{
    if (m_activeButton == 25) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_6_pressed()
{
    m_activeButton = 26;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_6_released()
{
    if (m_activeButton == 26) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_7_pressed()
{
    m_activeButton = 27;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_7_released()
{
    if (m_activeButton == 27) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_8_pressed()
{
    m_activeButton = 28;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_8_released()
{
    if (m_activeButton == 28) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_9_pressed()
{
    m_activeButton = 29;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_9_released()
{
    if (m_activeButton == 29) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_10_pressed()
{
    m_activeButton = 30;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_10_released()
{
    if (m_activeButton == 30) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_11_pressed()
{
    m_activeButton = 31;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_11_released()
{
    if (m_activeButton == 31) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_12_pressed()
{
    m_activeButton = 32;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_12_released()
{
    if (m_activeButton == 32) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_13_pressed()
{
    m_activeButton = 33;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_13_released()
{
    if (m_activeButton == 33) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_14_pressed()
{
    m_activeButton = 34;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_14_released()
{
    if (m_activeButton == 34) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_15_pressed()
{
    m_activeButton = 35;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_15_released()
{
    if (m_activeButton == 35) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_16_pressed()
{
    m_activeButton = 36;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_16_released()
{
    if (m_activeButton == 36) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_17_pressed()
{
    m_activeButton = 37;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_17_released()
{
    if (m_activeButton == 37) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_18_pressed()
{
    m_activeButton = 38;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_18_released()
{
    if (m_activeButton == 38) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_19_pressed()
{
    m_activeButton = 39;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_19_released()
{
    if (m_activeButton == 39) {
        m_activeButton = 0;
        m_timer->stop();
    }
}

void MainWindow::on_pushButton_2_20_pressed()
{
    m_activeButton = 40;
    if (!m_timer->isActive()) {
        m_timer->start();
    }
}


void MainWindow::on_pushButton_2_20_released()
{
    if (m_activeButton == 40) {
        m_activeButton = 0;
        m_timer->stop();
    }
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
                           QString frame = ui->frameLineEdit->text();
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
    delete ui;
    if (rclcpp::ok()) {
        rclcpp::shutdown();
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
    QString frame_id = ui->frameLineEdit->text();

    // Initialize the grid display
    grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
    if (grid_)
    {
        grid_->subProp("Line Style")->setValue("Lines");
        grid_->subProp("Color")->setValue(QColor(Qt::white));
        grid_->subProp("Reference Frame")->setValue(frame_id);
        // qDebug() << "Grid display configured for fixed frame:" << frame_id;
    }
    else
    {
        // qDebug() << "Failed to create Grid display.";
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
        robot_model_display_->subProp("Description Topic")->setValue("/robot_description"); // Set the topic to /robot_description
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
    QString frame_id = ui->frameLineEdit->text();
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
