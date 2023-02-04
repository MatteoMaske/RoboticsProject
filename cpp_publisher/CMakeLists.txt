cmake_minimum_required(VERSION 3.0.2)
project(cpp_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 3.3 REQUIRED)

add_message_files(
  FILES
  Coordinates.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(move src/move.cpp)
add_executable(planner src/planner.cpp)

target_link_libraries(move ${catkin_LIBRARIES})
install(TARGETS move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(planner ${catkin_LIBRARIES})
install(TARGETS planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(move $(catkin_EXPORTED_TARGETS))
add_dependencies(planner $(catkin_EXPORTED_TARGETS))

