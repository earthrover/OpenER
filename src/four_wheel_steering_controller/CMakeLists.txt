cmake_minimum_required(VERSION 2.8.3)
project(four_wheel_steering_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(${PROJECT_NAME}_CATKIN_DEPS
    controller_interface
    nav_msgs
    four_wheel_steering_msgs
    realtime_tools
    tf
    urdf_geometry_parser)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/four_wheel_steering_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES four_wheel_steering_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(catkin REQUIRED COMPONENTS std_srvs controller_manager tf)

add_executable(four_wheel_steering src/four_wheel_steering.cpp)
target_link_libraries(four_wheel_steering ${catkin_LIBRARIES})

add_dependencies(tests four_wheel_steering)
