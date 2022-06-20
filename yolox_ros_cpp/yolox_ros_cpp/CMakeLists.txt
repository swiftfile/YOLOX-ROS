cmake_minimum_required(VERSION 3.5)
project(yolox_ros_cpp)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(OpenCV REQUIRED)


# find dependencies
find_package(yolox_cpp)
if(NOT yolox_cpp_FOUND)
    message(WARNING "skipping yolox_ros_cpp, no yolox_cpp found")
    return()
endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        cv_bridge
        std_msgs
        image_transport
        bboxes_ex_msgs
        OpenCV
        nodelet
        yolox_cpp
        )
#        catkin_package(
#        CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs
#)
if (NOT yolox_cpp_FOUND)
    message(WARNING "skipping yolox_ros_cpp, no yolox_cpp found")
    return()
endif ()


include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS} include)

add_executable(yolox_ros_cpp
        src/yolox_ros_cpp.cpp
        include/yolox_ros_cpp/yolox_ros_cpp.hpp
#        ../yolox_cpp/src/yolox_tensorrt.cpp
#        ../yolox_cpp/include/yolox_cpp/yolox_tensorrt.hpp
        )

add_dependencies(yolox_ros_cpp
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
        )

target_compile_options(yolox_ros_cpp PUBLIC -Wall)

target_link_libraries(yolox_ros_cpp
        ${catkin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${OpenCV_LIBRARIES}
#        yolox_cpp
        )


# executable


