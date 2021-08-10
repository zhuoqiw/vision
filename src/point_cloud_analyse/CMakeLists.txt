cmake_minimum_required(VERSION 3.5)
project(point_cloud_analyse)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(PCL 1.3 REQUIRED)

add_library(point_cloud_analyse SHARED src/point_cloud_analyse.cpp)

target_include_directories(point_cloud_analyse
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(point_cloud_analyse
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(point_cloud_analyse ${PCL_LIBRARIES})

rclcpp_components_register_node(point_cloud_analyse
  PLUGIN "point_cloud_analyse::PointCloudAnalyse"
  EXECUTABLE point_cloud_analyse_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS point_cloud_analyse EXPORT export_point_cloud_analyse DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_point_cloud_analyse HAS_LIBRARY_TARGET)

ament_package()
