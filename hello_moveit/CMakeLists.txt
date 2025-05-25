cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(shape_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)


add_executable(simple_planning src/simple_planning.cpp)
target_include_directories(simple_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  simple_planning
  "moveit_ros_planning_interface"
  "rviz_visual_tools"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(joint_to_ee_pose src/joint_to_ee_pose.cpp)
ament_target_dependencies(joint_to_ee_pose
  "rclcpp"
  "trajectory_msgs"
  "moveit_ros_planning_interface"
)

add_executable(scene_initializer src/scene_initializer.cpp)
ament_target_dependencies(scene_initializer
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_msgs"          
  "shape_msgs"
  "geometry_msgs"
  "geometric_shapes"
  )


install(TARGETS
  simple_planning
  joint_to_ee_pose
  scene_initializer     
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
