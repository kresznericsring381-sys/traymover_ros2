cmake_minimum_required(VERSION 3.5)
project(traymover_robot_nav)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(dependencies
  geometry_msgs
  nav2_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
)

add_library(${PROJECT_NAME}_progress_checker SHARED
  src/yaw_progress_checker.cpp
)
ament_target_dependencies(${PROJECT_NAME}_progress_checker ${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pcd2pgm
    test/test_pcd2pgm.py
  )
  ament_add_pytest_test(
    test_fastlio_online_map_filter
    test/test_fastlio_online_map_filter.py
  )
  ament_add_pytest_test(
    test_pcd_clean
    test/test_pcd_clean.py
  )
  ament_add_pytest_test(
    test_nav_simple_config
    test/test_nav_simple_config.py
  )
endif()

install(
  DIRECTORY launch config map rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  TARGETS ${PROJECT_NAME}_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME}_progress_checker)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()
