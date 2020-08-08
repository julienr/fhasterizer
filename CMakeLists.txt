cmake_minimum_required(VERSION 3.4)
project(fhasterizer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(SDL2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wfatal-errors -Werror")

include_directories(${PROJECT_SOURCE_DIR}/third_party/eigen-3.3.7)
# Disable vectorization to simplify our lives:
# http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html#getrid
add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

add_executable(rasterizer src/main.cpp)
include_directories(${SDL2_INCLUDE_DIR})
target_link_libraries(rasterizer ${SDL2_LIBRARY})
