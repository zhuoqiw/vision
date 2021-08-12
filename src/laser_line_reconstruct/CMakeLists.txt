cmake_minimum_required(VERSION 3.5)
project(laser_line_reconstruct)

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
find_package(sensor_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common)

add_library(laser_line_reconstruct SHARED src/laser_line_reconstruct.cpp)

target_include_directories(laser_line_reconstruct
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(laser_line_reconstruct
  rclcpp
  rclcpp_components
  sensor_msgs
  shared_interfaces
)

if(opencv_world IN_LIST OpenCV_LIBS)
  target_link_libraries(laser_line_reconstruct opencv_world)
else()
  target_link_libraries(laser_line_reconstruct ${OpenCV_LIBS})
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(laser_line_reconstruct ${PCL_LIBRARIES})

rclcpp_components_register_node(laser_line_reconstruct
  PLUGIN "laser_line_reconstruct::LaserLineReconstruct"
  EXECUTABLE laser_line_reconstruct_node
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

install(TARGETS laser_line_reconstruct EXPORT export_laser_line_reconstruct DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_laser_line_reconstruct HAS_LIBRARY_TARGET)

ament_package()
