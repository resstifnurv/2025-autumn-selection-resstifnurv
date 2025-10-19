cmake_minimum_required(VERSION 3.8)
project(virtual_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(camera_node SHARED
    src/camera_node.cpp
    src/VirtualCamera.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(OpenCV REQUIRED)
target_link_libraries(camera_node
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(
    camera_node
    PLUGIN "CameraNode"
    EXECUTABLE camera
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config
)
