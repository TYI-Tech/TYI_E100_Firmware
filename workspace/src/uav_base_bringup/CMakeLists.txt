cmake_minimum_required(VERSION 3.0.2)
project(uav_base_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  rosgraph_msgs
  sensor_msgs
  tf2_ros
)

add_message_files(FILES EscRpm.msg)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS
  scripts/shared_timebase_clock.py
  scripts/high_rate_odom_ekf.py
  scripts/mavros_status_control.py
  scripts/px4_esc_rpm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/internal_timebase.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/wait_for_livox_ready.sh
  scripts/wait_for_lio_ready.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
