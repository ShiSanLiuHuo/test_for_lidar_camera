cmake_minimum_required(VERSION 3.5)
project(camera)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加 camera_node 可执行文件
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(camera_node ${OpenCV_LIBS})

# 添加 sub_node 可执行文件
add_executable(sub_node src/sub.cpp)
ament_target_dependencies(sub_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(sub_node ${OpenCV_LIBS})

# 添加 webm_camera_node 可执行文件
add_executable(webm_camera_node src/webm_camera_node.cpp)
ament_target_dependencies(webm_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(webm_camera_node ${OpenCV_LIBS})

# 添加 transform_node 可执行文件
add_executable(transform_node src/transform_compressedImg2Img.cpp)
ament_target_dependencies(transform_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(transform_node ${OpenCV_LIBS})

# 安装所有可执行文件
install(TARGETS
  camera_node
  sub_node
  webm_camera_node
  transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出所有依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_package()
