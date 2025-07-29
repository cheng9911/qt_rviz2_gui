import os
from launch import LaunchDescription
from launch . substitutions import LaunchConfiguration
from launch_ros . actions import Node
from launch_ros . substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'qt6_ros2_template'
    urdf_name = "robot1.urdf"
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name). find(package_name)
    urdf_model_path = os . path . join(pkg_share, f'urdf/{urdf_name }')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )
    joint_state_publisher_node = Node (
    package ='joint_state_publisher_gui',
    executable ='joint_state_publisher_gui',
    name ='joint_state_publisher_gui',
    arguments =[ urdf_model_path ]
    )

    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )
    qt_rviz2_gui_node = Node(
        package=package_name,
        executable='qt6_ros2_template',
        name='qt6_ros2_template',
        output='screen',
        parameters=[
            {'urdf_model_path': urdf_model_path},
            {'base': 'base_link'},
            {'tip': 'link_7'},
            {'sim': True}]  # 传递布尔参数
        # 如果有参数，可以用 parameters=[] 传入
    )
    ld.add_action(robot_state_publisher_node)
    #ld . add_action ( joint_state_publisher_node )
    ld.add_action(qt_rviz2_gui_node)
    # ld . add_action(rviz2_node)
    return ld
