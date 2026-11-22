cmake_minimum_required(VERSION 3.22)
project(basalt_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT BASALT_SOURCE_DIR)
  foreach(candidate ../thirdparty/basalt ../../thirdparty/basalt ../../../thirdparty/basalt)
    get_filename_component(path ${CMAKE_CURRENT_SOURCE_DIR}/${candidate} ABSOLUTE)
    if(EXISTS ${path}/src/vi_estimator/sqrt_keypoint_vio.cpp)
      set(BASALT_SOURCE_DIR ${path})
      break()
    endif()
  endforeach()
endif()
if(NOT EXISTS ${BASALT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vio.cpp)
  message(FATAL_ERROR "basalt sources not found next to the workspace thirdparty folder, set -DBASALT_SOURCE_DIR=<path>")
endif()
message(STATUS "basalt sources: ${BASALT_SOURCE_DIR}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

include(cmake/basalt_core.cmake)

add_library(basalt_vio SHARED src/basalt_vio.cpp)
target_include_directories(basalt_vio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(basalt_vio PUBLIC
  basalt_core
  rclcpp::rclcpp
  message_filters::message_filters
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

add_executable(basalt_vio_node src/basalt_vio_main.cpp)
target_link_libraries(basalt_vio_node basalt_vio)

install(TARGETS basalt_vio
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS basalt_vio_node DESTINATION lib/${PROJECT_NAME})

ament_package()
