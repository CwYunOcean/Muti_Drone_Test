cmake_minimum_required(VERSION 3.8)
project(fastlio2_to_ego_swarm_leveling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_transform
  src/leveled_frame_transform.cpp
)
target_include_directories(${PROJECT_NAME}_transform
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_transform
  nav_msgs
  pcl_conversions
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_transform
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/fastlio2_to_ego_swarm_leveling_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_transform
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_transform ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(
    test_leveled_frame_transform
    test/test_leveled_frame_transform.cpp
  )
  target_link_libraries(test_leveled_frame_transform
    ${PROJECT_NAME}_transform
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )

  ament_add_pytest_test(
    test_config_contract
    test/test_config_contract.py
  )

  ament_add_pytest_test(
    test_node_contract
    test/test_node_contract.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_transform)
ament_export_dependencies(Eigen3)

ament_package()
