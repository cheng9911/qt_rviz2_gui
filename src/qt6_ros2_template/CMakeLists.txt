cmake_minimum_required(VERSION 3.10)  # 提升版本以匹配第一个CMakeLists
project(qt6_ros2_template VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  # 统一输出目录

# 添加rocos和grpc的路径
list(APPEND CMAKE_PREFIX_PATH /opt/rocos/app)
list(APPEND CMAKE_PREFIX_PATH /opt/grpc)
include_directories(/opt/grpc/include)  # 添加grpc头文件路径

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 控制是否使用 ROS 的选项
option(USE_ROS "Build with ROS2 support" ON)

# 检查是否在 ROS 环境中
if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "ROS environment detected: $ENV{ROS_DISTRO}")
    set(USE_ROS ON)
else()
    message(STATUS "ROS environment not detected. Building without ROS support.")
    set(USE_ROS OFF)
endif()

# 查找Qt6
# if (UNIX AND NOT APPLE)
#     list(APPEND CMAKE_PREFIX_PATH "/opt/Qt/6.6.2/gcc_64")
# endif ()
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 COMPONENTS Widgets QUIET)
find_package(nav_msgs REQUIRED)
set(USE_QT ${Qt5_FOUND})

# 查找rocos和其他依赖库
find_package(rocos_app REQUIRED)
find_package(Eigen3 REQUIRED)
#add_subdirectory(3rdparty/gflags)  # 添加gflags子目录
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# 添加3rdparty依赖
# add_subdirectory(3rdparty/kdl_parser)
#add_subdirectory(3rdparty/ruckig)

# 包含目录
include_directories(
    ${LIBMODBUS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接目录
link_directories(${LIBMODBUS_LIBRARY_DIRS})

# ROS 2 依赖
if(USE_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)

endif()

# Qt 相关设置
if(USE_QT)
    message(STATUS "Qt5 found. Building with Qt support.")
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h *.hpp)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp *.c)
    file(GLOB UI_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
    file(GLOB RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

    add_executable(${PROJECT_NAME}
        ${HEADERS}
        ${SOURCES}
        ${UI_FILES}
        ${RESOURCES}
    )

    # 链接rocos相关库和其他依赖，与第一个CMakeLists保持一致
    target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        #gflags::gflags
        # kdl_parser
        rocos::hardware
        rocos::kinematics
        rocos::drive
        rocos::protocol
        rocos::robot
        Eigen3::Eigen
        ${LIBMODBUS_LIBRARIES}
    )

    if(USE_ROS)
        ament_target_dependencies(${PROJECT_NAME}
            rclcpp
            std_msgs
            sensor_msgs
            rclcpp_components
            rviz_common
            rviz_default_plugins
            rviz_ogre_vendor
            rviz_rendering
            nav_msgs
        )
    endif()

    # 为 Windows 设置 GUI 应用属性
    if(WIN32)
        set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()

    # 为 macOS 设置 bundle 属性
    if(APPLE)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()

    # 复制配置文件，与第一个CMakeLists保持一致
    file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/public.key config/license)
    file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin/config)
else()
    message(WARNING "Qt6 not found. Building without Qt support.")
endif()

# 安装目标
if(USE_ROS)
    install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
    )

    # 安装启动文件
    install(DIRECTORY
        launch
        urdf
        meshes
        DESTINATION share/${PROJECT_NAME}
    )

    ament_package()
else()
    if(USE_QT)
        install(TARGETS ${PROJECT_NAME}
            BUNDLE DESTINATION .
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )

        # 安装配置文件
        install(DIRECTORY ${CMAKE_BINARY_DIR}/bin/config
            DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    else()
        message(WARNING "Neither ROS nor Qt support available. No targets will be built.")
    endif()
endif()

# 添加编译定义
if(USE_ROS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ROS)
endif()
