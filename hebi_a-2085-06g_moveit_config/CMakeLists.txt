cmake_minimum_required(VERSION 3.22)
project(hebi_a-2085-06g_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(hebi_arm_mover1 src/hebi_arm_mover1.cpp)
ament_target_dependencies(hebi_arm_mover1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_mover2 src/hebi_arm_mover2.cpp)
ament_target_dependencies(hebi_arm_mover2
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_mover3 src/hebi_arm_mover3.cpp)
ament_target_dependencies(hebi_arm_mover3
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_mover4 src/hebi_arm_mover4.cpp)
ament_target_dependencies(hebi_arm_mover4
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_j6_rotator1 src/hebi_j6_rotator1.cpp)
ament_target_dependencies(hebi_j6_rotator1
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_j6_rotator2 src/hebi_j6_rotator2.cpp)
ament_target_dependencies(hebi_j6_rotator2
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_j6_rotator3 src/hebi_j6_rotator3.cpp)
ament_target_dependencies(hebi_j6_rotator3
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_j6_rotator4 src/hebi_j6_rotator4.cpp)
ament_target_dependencies(hebi_j6_rotator4
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_j6_rotator5 src/hebi_j6_rotator5.cpp)
ament_target_dependencies(hebi_j6_rotator5
  rclcpp
  moveit_ros_planning_interface
  std_msgs
) 

add_executable(hebi_j6_rotatorno src/hebi_j6_rotatorno.cpp)
ament_target_dependencies(hebi_j6_rotatorno
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_home_position src/hebi_home_position.cpp)
ament_target_dependencies(hebi_home_position
  rclcpp
  std_msgs
  moveit_ros_planning_interface
)

add_executable(hebi_arm_moverU0 src/hebi_arm_moverU0.cpp)
ament_target_dependencies(hebi_arm_moverU0
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_moverU1 src/hebi_arm_moverU1.cpp)
ament_target_dependencies(hebi_arm_moverU1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_moverU2 src/hebi_arm_moverU2.cpp)
ament_target_dependencies(hebi_arm_moverU2
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_moverU3 src/hebi_arm_moverU3.cpp)
ament_target_dependencies(hebi_arm_moverU3
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_arm_moverU4 src/hebi_arm_moverU4.cpp)
ament_target_dependencies(hebi_arm_moverU4
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_j6_rotatorU1 src/hebi_j6_rotatorU1.cpp)
ament_target_dependencies(hebi_j6_rotatorU1
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_j6_rotatorU2 src/hebi_j6_rotatorU2.cpp)
ament_target_dependencies(hebi_j6_rotatorU2
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_j6_rotatorU3 src/hebi_j6_rotatorU3.cpp)
ament_target_dependencies(hebi_j6_rotatorU3
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

add_executable(hebi_bell_home src/hebi_bell_home.cpp)
ament_target_dependencies(hebi_bell_home
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_bell_ready src/hebi_bell_ready.cpp)
ament_target_dependencies(hebi_bell_ready
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_bell_approach src/hebi_bell_approach.cpp)
ament_target_dependencies(hebi_bell_approach
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_bell_grasp src/hebi_bell_grasp.cpp)
ament_target_dependencies(hebi_bell_grasp
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_bell_roll src/hebi_bell_roll.cpp)
ament_target_dependencies(hebi_bell_roll
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_bell_place src/hebi_bell_place.cpp)
ament_target_dependencies(hebi_bell_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

add_executable(hebi_bell_retreat src/hebi_bell_retreat.cpp)
ament_target_dependencies(hebi_bell_retreat
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

install(TARGETS
  hebi_arm_mover1
  hebi_arm_mover2
  hebi_arm_mover3
  hebi_arm_mover4
  hebi_j6_rotator1
  hebi_j6_rotator2
  hebi_j6_rotator3
  hebi_j6_rotator4
  hebi_j6_rotator5
  hebi_j6_rotatorno
  hebi_arm_moverU0
  hebi_arm_moverU1
  hebi_arm_moverU2
  hebi_arm_moverU3
  hebi_arm_moverU4
  hebi_j6_rotatorU1
  hebi_j6_rotatorU2
  hebi_j6_rotatorU3
  hebi_home_position
  hebi_bell_home
  hebi_bell_ready
  hebi_bell_approach
  hebi_bell_grasp
  hebi_bell_roll
  hebi_bell_place
  hebi_bell_retreat
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME}) 

ament_package()
