cmake_minimum_required(VERSION 3.10)

project(downsamplePC)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(downsamplePC main.cpp ${CMAKE_SOURCE_DIR}/src/downsample.cpp)
# target_include_directories(downsamplePC PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(downsamplePC ${PCL_LIBRARIES} yaml-cpp )
