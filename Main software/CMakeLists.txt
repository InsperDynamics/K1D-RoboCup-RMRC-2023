cmake_minimum_required(VERSION 3.16.3)

project(K1D)

find_package(OpenCV REQUIRED)
find_package(SDL2 2.26.3 EXACT REQUIRED PATHS /usr/local/include/SDL2)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  genmsg
  cv_bridge
  image_transport
  open_manipulator_msgs
)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(K1D K1D_MAIN.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED IMPORTED_TARGET zbar)
target_link_libraries(K1D PUBLIC PkgConfig::ZBar)

target_link_libraries(K1D PUBLIC ${OpenCV_LIBS})
target_link_libraries(K1D PUBLIC ${ZBAR_LIBRARIES} zbar)
target_link_libraries(K1D PUBLIC ${SDL2_LIBRARIES})
target_link_libraries(K1D PUBLIC ${catkin_LIBRARIES})
