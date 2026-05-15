cmake_minimum_required(VERSION 3.5)
project(traymover_robot_nav)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

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

ament_package()
