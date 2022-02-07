cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

project(car_simulator)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(car_simulator main.cpp)

# linking
target_link_libraries(car_simulator ${OpenCV_LIBS})
