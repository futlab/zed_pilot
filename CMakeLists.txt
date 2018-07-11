cmake_minimum_required(VERSION 3.1)
project(zed_pilot)

option(USE_ROS "Use ROS" ON)
option(SHOW_RESULT "Show result" OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(${SHOW_RESULT})
  add_definitions(-DSHOW_RESULT)
endif()

##############
## Packages ##
##############

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

if (${USE_ROS})
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        stereo_msgs
        cv_bridge
        mavros_msgs
        image_transport
        tf2_ros
    )
    catkin_package(
        INCLUDE_DIRS include
    #  LIBRARIES autopilot
        CATKIN_DEPENDS mavros_msgs roscpp rospy std_msgs sensor_msgs stereo_msgs cv_bridge
    #  DEPENDS system_lib
    )
endif()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  include
)

set(SOURCES
    src/zedpilot.cpp        include/zedpilot.h
    src/pilot.cpp           include/pilot.h
    src/controllers.cpp     include/controllers.h
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

set(LIBRARIES
    ${ZED_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}
)

if (${USE_ROS})
    add_executable(pilot_node
        src/node.cpp
        ${SOURCES}
    )
    target_link_libraries(pilot_node
        ${catkin_LIBRARIES}
        ${LIBRARIES}
    )
else()
    add_executable(pilot
        src/main.cpp
        ${SOURCES}
    )
    target_link_libraries(pilot
        ${LIBRARIES}
    )
endif()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autopilot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)