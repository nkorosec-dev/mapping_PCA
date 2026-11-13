cmake_minimum_required(VERSION 3.16)
project(mapping_framework LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project paths
get_filename_component(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(DATA_DIR "${ROOT}/data/generated")
file(MAKE_DIRECTORY "${DATA_DIR}")

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Synthetic mapping
add_executable(mapping main.cpp mapping.cpp)
target_link_libraries(mapping PRIVATE Eigen3::Eigen)

# OpenCV experiments
add_executable(opencv_OV opencv_OV.cpp)
target_link_libraries(opencv_OV PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(opencv_OV PRIVATE ${OpenCV_INCLUDE_DIRS})

# RealSense depth to point cloud
add_executable(depth_to_cloud depth_to_cloud.cpp mapping.cpp)
target_link_libraries(depth_to_cloud PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(depth_to_cloud PRIVATE ${OpenCV_INCLUDE_DIRS})

# Synthetic experiment + visualization
add_custom_target(plot
    COMMAND $<TARGET_FILE:mapping>
    COMMAND "${ROOT}/.venv/bin/python" "${ROOT}/scripts/plot.py"
    WORKING_DIRECTORY "${DATA_DIR}"
    DEPENDS mapping
    USES_TERMINAL
)

# Real depth dataset
add_custom_target(real_cloud
    COMMAND $<TARGET_FILE:depth_to_cloud>
            "${ROOT}/Dataset/depth/45deg_full_board_D6-orange_L5-black_L3-white_00000.png"
    WORKING_DIRECTORY "${DATA_DIR}"
    DEPENDS depth_to_cloud
    USES_TERMINAL
)