project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/Helper.cpp src/Helper.h src/Path.cpp src/Path.h src/Vehicle.cpp src/Vehicle.h src/spline.h src/constants.h src/Road.cpp src/Road.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

#find_package(Boost REQUIRED COMPONENTS system iostreams)
#include_directories( ${Boost_INCLUDE_DIRS} )
#link_directories( ${Boost_LIBRARY_DIRS})

include_directories(Eigen-3.3)
#include_directories(gnuplot-iostream)

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
