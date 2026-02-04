cmake_minimum_required(VERSION 3.8)
project(telebot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(set_initial_pose
  src/set_initial_pose.cpp
)

add_executable(go_to_goal
src/go_to_goal.cpp
)

ament_target_dependencies(go_to_goal
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)


ament_target_dependencies(
  set_initial_pose
  rclcpp
  geometry_msgs
)

install(DIRECTORY launch params config src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS go_to_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  set_initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
