cmake_minimum_required(VERSION 3.0.2)
project(multi_box_filter)

set(CMAKE_CXX_STANDARD 11)
##############################################################################
# Find dependencies
##############################################################################
set(THIS_PACKAGE_ROS_DEPS sensor_msgs roscpp tf filters message_filters
  laser_geometry pluginlib angles)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


##############################################################################
# Define package
##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_box_filter
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
  )


##############################################################################
# Build
##############################################################################
add_library(multi_box_filter
  src/multi_box_filter.cpp
)
target_link_libraries(multi_box_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
target_link_libraries(scan_to_scan_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})


##############################################################################
# Install
##############################################################################
install(TARGETS multi_box_filter 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_filter_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)