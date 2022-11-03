cmake_minimum_required(VERSION 3.0.2)
project(robotnik_eipscanner)

## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rcomponent
  robotnik_msgs
  eipscanner_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotnik_eipscanner
  CATKIN_DEPENDS roscpp rcomponent robotnik_msgs eipscanner_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# EIPScanner
add_executable(eipscanner_node src/Ps_C1_C100_Input.cpp src/main.cpp src/ src/EIPScanner.cpp)

add_dependencies(eipscanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(eipscanner_node
  EIPScanner
  ${catkin_LIBRARIES}
)

install(TARGETS eipscanner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testing-test launch/gtest.test tests/tests_1.cpp)
  target_link_libraries(testing-test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()