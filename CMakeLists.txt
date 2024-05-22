cmake_minimum_required(VERSION 3.8)
project(liquid_pickup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(rclcpp_action REQUIRED)

# add_library(action_client SHARED
#   src/navigation_behaviors.cpp)
# target_include_directories(action_client PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(action_client
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# ament_target_dependencies(action_client
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components")
# rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)

# install(TARGETS
#   action_client
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

include_directories(include ${CMAKE_SOURCE_DIR}/../BehaviorTree.CPP/src) 

set(BT_SOURCES
    src/navigation_behaviors.cpp 
    # src/vision_behaviors.cpp
    src/manipulator.cpp
    src/manipulator_behaviors.cpp
    # src/gripper_behavior.cpp
    src/robot.cpp
    src/path_queue.cpp
    src/helper.cpp
)

# add_library(pathqueue src/path_queue.cpp)

# target_link_libraries(pathqueue ${rclcpp_LIBRARIES} rclcpp behaviortree_cpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(liquid_pickup_node src/bt_node.cpp ${BT_SOURCES})
ament_target_dependencies(liquid_pickup_node rclcpp behaviortree_cpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs yaml-cpp)

install(TARGETS
    liquid_pickup_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
