cmake_minimum_required(VERSION 2.8.3)
project(ros_vino)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(InferenceEngine 2.0)
if (NOT InferenceEngine_FOUND)
    message(FATAL_ERROR "")
endif()

if(TARGET IE::ie_cpu_extension)
    add_definitions(-DWITH_EXTENSIONS)
endif()

find_package(Armadillo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  InferenceEngine
  std_msgs
  sensor_msgs
  roslaunch
  message_generation
  cv_bridge
  roslib
)

# Find OpenCV libray if exists
find_package(
  OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  Objects.msg
  Object.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  message_runtime
  cv_bridge
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${InferenceEngine_INCLUDE_DIRS}
  ${InferenceEngine_INCLUDE_DIRS}/../samples/common
  ${InferenceEngine_INCLUDE_DIRS}/../src/extension
  ${ARMADILLO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_detection_ssd
  src/object_detection_ssd/object_detection_ssd.cpp
  src/object_detection_ssd/main.cpp
)
add_dependencies(object_detection_ssd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_detection_ssd
  ${catkin_LIBRARIES}
  ${InferenceEngine_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
if(TARGET IE::ie_cpu_extension)
    target_link_libraries(object_detection_ssd 
    IE::ie_cpu_extension
    )
endif()

# target_include_directories(${IE_SAMPLE_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")

install(TARGETS object_detection_ssd
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(multi_kf_tracker
  src/multi_kf_tracker/multi_kf_tracker.cpp
  src/multi_kf_tracker/main.cpp
)
add_dependencies(multi_kf_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_kf_tracker
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
if(TARGET IE::ie_cpu_extension)
    target_link_libraries(multi_kf_tracker 
    IE::ie_cpu_extension
    )
endif()

# target_include_directories(${IE_SAMPLE_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")

install(TARGETS multi_kf_tracker
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# install(DIRECTORY launch/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

install(DIRECTORY models/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models )
   
install(DIRECTORY msg/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg )
   
install(DIRECTORY rviz/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz )
   
install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src )
