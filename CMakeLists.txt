cmake_minimum_required(VERSION 2.8.3)
project(ar_marker_client)

find_package(catkin REQUIRED COMPONENTS roscpp ar_track_alvar geometry_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/MarkerOffset.cfg
)

catkin_package(
  INCLUDE_DIRS launch
)

add_library(MarkerTracker src/MarkerTracker.cpp)
target_link_libraries(MarkerTracker ${catkin_LIBRARIES})

add_executable(marker_tracker_node src/marker_tracker_node.cpp)
target_link_libraries(marker_tracker_node MarkerTracker ${catkin_LIBRARIES})

#add_library(KalmanTracker src/KalmanTracker.cpp)
#add_dependencies(KalmanTracker ${PROJECT_NAME}_gencfg)
#target_link_libraries(KalmanTracker ${catkin_LIBRARIES})

#add_executable(kalman_tracker_node src/kalman_tracker_node.cpp)
#add_dependencies(kalman_tracker_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(kalman_tracker_node KalmanTracker ${catkin_LIBRARIES})
