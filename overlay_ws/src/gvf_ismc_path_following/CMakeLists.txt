cmake_minimum_required(VERSION 3.8)
project(gvf_ismc_path_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gvf_path_following_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/three_leaf_gvf.cpp
  src/ismc_outer_loop.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  gvf_path_following_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)

add_executable(gvf_reference_node
  src/gvf_reference_node.cpp
)
target_link_libraries(gvf_reference_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(gvf_reference_node
  geometry_msgs
  gvf_path_following_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
  visualization_msgs
)

add_executable(ismc_velocity_tracker_node
  src/ismc_velocity_tracker_node.cpp
)
target_link_libraries(ismc_velocity_tracker_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(ismc_velocity_tracker_node
  geometry_msgs
  gvf_path_following_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    gvf_reference_node
    ismc_velocity_tracker_node
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
    test_three_leaf_gvf
    test/test_three_leaf_gvf.cpp
  )
  target_link_libraries(test_three_leaf_gvf
    ${PROJECT_NAME}_core
    Eigen3::Eigen
  )

  ament_add_pytest_test(
    test_config_contract
    test/test_config_contract.py
  )

  ament_add_gtest(
    test_ismc_outer_loop
    test/test_ismc_outer_loop.cpp
  )
  target_link_libraries(test_ismc_outer_loop
    ${PROJECT_NAME}_core
    Eigen3::Eigen
  )

  ament_add_pytest_test(
    test_node_contract
    test/test_node_contract.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  gvf_path_following_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
  visualization_msgs
)

ament_package()
