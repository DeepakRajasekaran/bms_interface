cmake_minimum_required(VERSION 3.0.2)
project(bms_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(micronix_bms_node src/MicronixBms.cpp)
target_link_libraries(micronix_bms_node ${catkin_LIBRARIES})

add_executable(atc_bms_node src/AtcBms.cpp)
target_link_libraries(atc_bms_node ${catkin_LIBRARIES})

add_executable(aperion_bms_node src/AperionBms.cpp)
target_link_libraries(aperion_bms_node ${catkin_LIBRARIES})

install(TARGETS micronix_bms_node atc_bms_node aperion_bms_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
