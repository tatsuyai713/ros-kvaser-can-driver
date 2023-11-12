cmake_minimum_required(VERSION 3.5)
project(ros_kvaser_can_driver)

# 必要なパッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)


# カスタムメッセージのインクルード
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CANFrame.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# インクルードディレクトリをエクスポート
ament_export_include_directories(include)
include_directories(include)
# Include the directories from yaml-cpp
include_directories(${YAML_CPP_INCLUDE_DIR})

# パッケージをビルドするための設定
ament_package()

# ビルドに必要なライブラリを指定
add_executable(kvaser_can_driver src/kvaser_can_driver.cpp src/kvaser_can_driver_node.cpp)
ament_target_dependencies(kvaser_can_driver rclcpp std_msgs yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kvaser_can_driver  ${YAML_CPP_LIBRARIES} canlib "${cpp_typesupport_target}") 

install(TARGETS 
  kvaser_can_driver
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

