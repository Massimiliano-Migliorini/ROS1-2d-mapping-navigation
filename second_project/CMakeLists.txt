cmake_minimum_required(VERSION 3.0.2)
project(second_project)

# -------------------------------------------------------------
#  Trova i pacchetti di cui dipendiamo a compile-time e run-time
# -------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
  actionlib
  move_base_msgs
  tf2_ros             
  tf2_geometry_msgs    
)
# Informa catkin che questo pacchetto non esporta librerie
catkin_package()

# -------------------------------------------------------------
#  Include directory del pacchetto + quelle dei dependenziati
# -------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ------------------- Eseguibili da compilare ------------------

#  -- odom_to_tf  : converte nav_msgs/Odometry in tf --------------

add_executable(data_conversion src/data_conversion.cpp)
target_link_libraries(data_conversion ${catkin_LIBRARIES})


#  -- goal_sequencer : invia sequenza di goal a move_base --------
# add_executable(goal_sequencer src/goal_sequencer.cpp)
# target_link_libraries(goal_sequencer ${catkin_LIBRARIES})
add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
