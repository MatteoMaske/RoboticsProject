cmake_minimum_required(VERSION 3.0.2)
project(image_transport_ws)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_transport_ws src/my_subscriber.cpp)


target_link_libraries(image_transport_ws ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS image_transport_ws
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)