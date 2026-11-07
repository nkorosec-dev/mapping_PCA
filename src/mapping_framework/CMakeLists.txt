cmake_minimum_required(VERSION 3.16)
project(mapping_framework LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(mapping main.cpp)
target_link_libraries(mapping PRIVATE Eigen3::Eigen)

add_executable(opencv_OV opencv_OV.cpp)
target_link_libraries(opencv_OV PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(opencv_OV PRIVATE ${OpenCV_INCLUDE_DIRS})

add_custom_target(plot
    COMMAND $<TARGET_FILE:mapping>
    COMMAND ${CMAKE_SOURCE_DIR}/../../.venv/bin/python
            ${CMAKE_SOURCE_DIR}/../../plot.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../..
    DEPENDS mapping
)