cmake_minimum_required(VERSION 3.8)
project(position_cmd_to_px4_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/bridge_input_builder.cpp
  src/offboard_control_mode_builder.cpp
  src/trajectory_setpoint_conversion.cpp
  src/bridge_state_machine.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME}_core
  px4_msgs
  quadrotor_msgs
)

add_executable(${PROJECT_NAME}_node
  src/position_cmd_to_px4_bridge_node.cpp
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME}_node
  nav_msgs
  px4_msgs
  quadrotor_msgs
  rclcpp
)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
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
    test_config_contract
    test/test_config_contract.cpp
  )
  ament_add_gtest(
    test_bridge_input_builder
    test/test_bridge_input_builder.cpp
  )
  target_link_libraries(test_bridge_input_builder
    ${PROJECT_NAME}_core
  )
  ament_target_dependencies(test_bridge_input_builder
    px4_msgs
  )
  ament_add_gtest(
    test_trajectory_setpoint_conversion
    test/test_trajectory_setpoint_conversion.cpp
  )
  target_link_libraries(test_trajectory_setpoint_conversion
    ${PROJECT_NAME}_core
    Eigen3::Eigen
  )
  ament_target_dependencies(test_trajectory_setpoint_conversion
    quadrotor_msgs
  )
  ament_add_gtest(
    test_bridge_state_machine
    test/test_bridge_state_machine.cpp
  )
  target_link_libraries(test_bridge_state_machine
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(
    test_offboard_control_mode_builder
    test/test_offboard_control_mode_builder.cpp
  )
  target_link_libraries(test_offboard_control_mode_builder
    ${PROJECT_NAME}_core
  )
  ament_target_dependencies(test_offboard_control_mode_builder
    px4_msgs
  )
  ament_add_gtest(
    test_px4_qos_profiles
    test/test_px4_qos_profiles.cpp
  )
  target_include_directories(test_px4_qos_profiles
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_px4_qos_profiles
    rclcpp
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3)

ament_package()
