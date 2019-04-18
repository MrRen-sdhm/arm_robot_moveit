cmake_minimum_required(VERSION 2.8.3)
project(pickup CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

set(OpenCV_DIR /usr/local/include/opencv)
set(CMAKE_CXX_STANDARD 11)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
 MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()


find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs message_generation tf sensor_msgs message_filters kinect2_bridge cv_bridge image_transport compressed_image_transport compressed_depth_image_transport )

# Opencv3.4.4配置
#find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs message_generation sensor_msgs message_filters kinect2_bridge)
#find_package(cv_bridge)
#find_package(image_transport)
#find_package(compressed_image_transport)
#find_package(compressed_depth_image_transport)

## System dependencies are found with CMake's conventions
# Opencv3.4.4配置
#find_package(OpenCV3.4.4 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL 1.7 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(OPENMP_FOUND)
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PickupObj.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES kinect2_viewer
         CATKIN_DEPENDS message_runtime
        #  DEPENDS system_lib
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
        SOURCES
        ${FILES_LIST}
        )

message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

include_directories(include
        ${catkin_INCLUDE_DIRS}
#        /opt/ros/kinetic/include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${kinect2_bridge_INCLUDE_DIRS}
        )

add_executable( single_thread_listener src/single_thread_listener.cpp)
target_link_libraries( single_thread_listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable( multi_thread_listener src/multi_thread_listener.cpp )
target_link_libraries( multi_thread_listener ${catkin_LIBRARIES})

add_executable( multi_topic_pub src/multi_topic_pub.cpp )
target_link_libraries( multi_topic_pub ${catkin_LIBRARIES})

add_executable(get_obj_depth_gpu src/get_obj_depth_gpu.cpp)
target_link_libraries(get_obj_depth_gpu
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${kinect2_bridge_LIBRARIES}
        ${image_transport_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        )

# Opencv3.4.4配置
#add_executable(get_obj_depth_cpu src/get_obj_depth_cpu.cpp)
#target_link_libraries(get_obj_depth_cpu
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBRARIES}
#        ${PCL_LIBRARIES}
#        ${kinect2_bridge_LIBRARIES}
#        ${image_transport_LIBRARIES}
#        ${cv_bridge_LIBRARIES}
#        )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Opencv3.4.4配置
## Mark executables and/or libraries for installation
#install(TARGETS get_obj_depth_cpu
#        #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinect2_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)CMakeLists.txt

