cmake_minimum_required(VERSION 3.1)
project(zed_pilot)

## Options ##

option(WITH_ROS "With ROS" ON)
option(WITH_GUI "Show result" OFF)
option(USE_GST "Use gstreamer" ON)
option(MSG_GEN "Generate message" OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(${WITH_ROS})
    add_definitions(-DWITH_ROS)
endif()

if(${WITH_GUI})
    add_definitions(-DWITH_GUI)
endif()

##############
## Packages ##
##############

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)

if(NOT ${Eigen3_INCLUDE_DIRS})
    find_package(Eigen3 REQUIRED)
endif()

#if(NOT ${Sophus_INCLUDE_DIRS}) find_package(Sophus REQUIRED) endif()

if (${WITH_ROS})
    if(${USE_GST})
        add_definitions(-DUSE_GST)
        find_package(PkgConfig)
        pkg_check_modules(GST REQUIRED
            gstreamer-1.0>=1.4
        )
    endif()
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        sensor_msgs
        stereo_msgs
        cv_bridge
        mavros_msgs
        image_transport
        tf2_ros
        ros2gst
    )
    if (${MSG_GEN})
        add_message_files(
           FILES
           ZedState.msg
        )
        generate_messages(
            #DEPENDENCIES
            #std_msgs
        )
    endif()

    catkin_package(
        INCLUDE_DIRS include
    #  LIBRARIES autopilot
        CATKIN_DEPENDS mavros_msgs roscpp rospy std_msgs sensor_msgs stereo_msgs cv_bridge message_runtime
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
  ${Sophus_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
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

if (${WITH_ROS})
    add_executable(pilot_node
        src/node.cpp
        ${SOURCES}
    )
    target_link_libraries(pilot_node
        ${catkin_LIBRARIES}
        ${LIBRARIES}
    )

else()
    #add_executable(pilot
    #    src/main.cpp
    #    ${SOURCES}
    #)
    #target_link_libraries(pilot
    #    ${LIBRARIES}
    #)
endif()

add_executable(arucogen
    src/arucogen.cpp
)
target_link_libraries(arucogen
    ${OpenCV_LIBRARIES}
)

add_executable(tester
    src/tester.cpp
    src/aruco.cpp           include/aruco.h
)
target_link_libraries(tester
    ${catkin_LIBRARIES}
    ${LIBRARIES}
)


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
