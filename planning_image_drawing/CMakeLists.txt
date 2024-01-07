cmake_minimum_required(VERSION 3.0.2)
project(planning_image_drawing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  set_img_flag.srv
  drawing_type.srv
  start_calculate_flag.srv
  start_drawing_flag.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES OpenCV
 CATKIN_DEPENDS roscpp std_msgs cv_bridge sensor_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_line_planner src/line_planner.cpp)
target_link_libraries(${PROJECT_NAME}_line_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(process_node
  src/main_process_node.cpp
)
target_link_libraries(process_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}_line_planner
)

add_executable(pub_image_node
  src/image_pub_test_node.cpp
)
target_link_libraries(pub_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(test_flag_node
  src/test_flag_node.cpp
)
target_link_libraries(test_flag_node
  ${catkin_LIBRARIES}
)