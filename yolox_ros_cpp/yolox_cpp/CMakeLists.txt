cmake_minimum_required(VERSION 3.5)
project(yolox_cpp)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
list(APPEND CUDA_NVCC_FLAGS "-std=c++11")
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_SEPARABLE_COMPILATION ON)

# build option
option(YOLOX_USE_OPENVINO "Use OpenVINO" OFF)
option(YOLOX_USE_TENSORRT "Use TensorRT" ON)

set(ENABLE_OPENVINO OFF)
set(YOLOX_USE_OPENVINO OFF)
set(ENABLE_TENSORRT ON)


# find dependencies
find_package(catkin REQUIRED COMPONENTS
        image_transport
        roscpp
        sensor_msgs
        cv_bridge
        # vision_msgs
        std_msgs
        )
catkin_package(
        CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs
)
include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
add_definitions(-DROS1 -DROS_DISTRO=${ROS_DISTRO})



add_library(yolox_cpp
        src/yolox_tensorrt.cpp
        include/yolox_cpp/yolox_tensorrt.hpp
#        src/yolox_openvino.cpp
        )


if (YOLOX_USE_TENSORRT)
    find_package(CUDA)
    find_library(NVINFER NAMES nvinfer)
    find_library(NVINFERPLUGIN NAMES nvinfer_plugin)
    find_library(NVPARSERS NAMES nvparsers)
    find_library(NVONNXPARSER NAMES nvonnxparser)
    find_library(NVONNXPARSERRUNTIME NAMES nvonnxparser_runtime)
    if (CUDA_FOUND AND NVINFER AND NVINFERPLUGIN AND NVPARSERS AND NVONNXPARSER)
        set(ENABLE_TENSORRT ON)
        set(SRC ${SRC} src/yolox_tensorrt.cpp)
    endif ()
endif ()

message(STATUS " ENABLE_OPENVINO: ${ENABLE_OPENVINO}")
message(STATUS " ENABLE_TENSORRT: ${ENABLE_TENSORRT}")

if (NOT ENABLE_OPENVINO AND NOT ENABLE_TENSORRT)
    message(WARNING "skip building yolox_cpp, no OpenVINO and TensorRT found")
    return()
endif ()
include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

install(TARGETS yolox_cpp
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(DIRECTORY
        DESTINATION share/${PROJECT_NAME}
)
install(
        DIRECTORY include/
        DESTINATION include
)


target_compile_options(yolox_cpp PUBLIC -Wall)
target_link_libraries(yolox_cpp
${CUDA_LIBRARIES}
${OpenCV_LIBRARIES}
nvinfer
nvinfer_plugin
nvparsers
nvonnxparser
# nvonnxparser_runtime
)




