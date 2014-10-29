cmake_minimum_required(VERSION 2.8.3)
project(gps_communication)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
  )

add_message_files(
	DIRECTORY msg
	FILES
	Coordinate.msg
	Coordinate_utm.msg
  CoordinateArray.msg
  SensorPacket.msg
  navdata_demo.msg
	)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps_communication
  CATKIN_DEPENDS ardrone_autonomy ardrone_autonomy-gps roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime
  DEPENDS system_lib
  )


include_directories( ${catkin_INCLUDE_DIRS} ${libgps_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} include ${catkin_INCLUDE_DIRS})

add_executable(main_function src/main_function.cpp)
add_executable(main_function_ok src/main_function_ok.cpp)
add_executable(main_function_stefano src/main_function_stefano.cpp)
add_executable(read_waypoints src/read_waypoints.cpp)

target_link_libraries(main_function ${catkin_LIBRARIES} ${libgps_LIBRARIES})
target_link_libraries(main_function_stefano ${catkin_LIBRARIES} ${libgps_LIBRARIES})
target_link_libraries(read_waypoints ${catkin_LIBRARIES} ${libgps_LIBRARIES})
target_link_libraries(main_function_ok ${catkin_LIBRARIES} ${libgps_LIBRARIES})