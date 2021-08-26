cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roslaunch
  roscpp
  rospy
  std_msgs
  std_srvs
  # three_dof_planar_manipulator
  rviz
  robot_state_publisher
  joint_state_publisher
  urdf
  message_generation
  rosserial_python
)

find_package(Boost REQUIRED COMPONENTS chrono)

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

add_message_files(
  FILES
  # EncoderInfo.msg 
  # Pwm.msg
  encoder_ticks_data.msg
  encoder_rad_data.msg
  pid_commands_data.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS hardware_interface controller_manager roscpp 
  rospy std_msgs std_srvs rviz robot_state_publisher 
  joint_state_publisher urdf message_runtime rosserial_python
#  DEPENDS system_lib
  DEPENDS Boost
)

include_directories(
 include
 ${Boost_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${YAMLCPP_INCLUDE_DIRS}
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_executable(wombotGen3_node
  src/wombotGen3_base.cpp
  src/wombotGen3_hardware.cpp
  src/encoder_mapping.cpp
)
target_link_libraries(wombotGen3_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAMLCPP_LIBRARIES})

roslaunch_add_file_check(launch)

# tells it that this target is dependent on the other targets exported by the same package
#  need that for messages
add_dependencies(wombotGen3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})