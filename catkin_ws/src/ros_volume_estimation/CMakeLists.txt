cmake_minimum_required(VERSION 3.0.2)
project(ros_volume_estimation)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate services
add_service_files(
  FILES
  ToggleProcessing.srv
)

## Generate messages dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs pcl_ros pcl_conversions message_runtime
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# No shared library needed for nodes

## Declare a C++ executable
add_executable(transform_node src/transform_node.cpp)
add_executable(crop_filter_node src/crop_filter_node.cpp)
add_executable(volume_estimator_node src/volume_estimator_node.cpp)
add_executable(workflow_control_node src/workflow_control_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(crop_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(volume_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(workflow_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(transform_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(crop_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(volume_estimator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(workflow_control_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS transform_node crop_filter_node volume_estimator_node workflow_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# No libraries to install

# No header files to install

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)