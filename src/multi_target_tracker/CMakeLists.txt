cmake_minimum_required(VERSION 3.5)
project(multi_target_tracker)

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

add_compile_options(-g)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(track SHARED
src/track.cpp
)

add_library(multi_target_tracker SHARED
src/multi_target_tracker.cpp
)

add_executable(multi_target_tracker_node src/multi_target_tracker_node.cpp)

add_executable(bbox_simulator_node src/bbox_simulator_node.cpp)

ament_target_dependencies(track rclcpp Eigen3)
ament_target_dependencies(multi_target_tracker rclcpp vision_msgs Eigen3)
ament_target_dependencies(multi_target_tracker_node rclcpp Eigen3 vision_msgs foxglove_msgs)
ament_target_dependencies(bbox_simulator_node rclcpp vision_msgs foxglove_msgs)

target_link_libraries(multi_target_tracker
  track
)

target_link_libraries(multi_target_tracker_node
  track
  multi_target_tracker
)
target_include_directories(track PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(multi_target_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(multi_target_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_libraries(track)
ament_export_libraries(multi_target_tracker)

install(TARGETS multi_target_tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS bbox_simulator_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  track
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  multi_target_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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

ament_package()
