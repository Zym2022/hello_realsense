cmake_minimum_required(VERSION 3.9)
project(pcl_test)

set(CMAKE_CXX_STANDARD 14)

find_package(realsense2 REQUIRED)

# pcl
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(-DBOOST_ALL_DYN_LINK)

# OpenMP
find_package(TBB REQUIRED)

add_executable(hello-realsense hello-realsense.cpp)
add_executable(shoot_templete shoot_templete.cpp)

target_link_libraries (hello-realsense pthread ${realsense2_LIBRARY} ${PCL_LIBRARIES} TBB::tbb)
target_link_libraries (shoot_templete pthread ${realsense2_LIBRARY} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS hello-realsense RUNTIME DESTINATION bin)

