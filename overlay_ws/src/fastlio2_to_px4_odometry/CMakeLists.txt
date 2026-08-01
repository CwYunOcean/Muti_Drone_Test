cmake_minimum_required(VERSION 3.8)
project(fastlio2_to_px4_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_conversion
  src/odometry_conversion.cpp
)
target_include_directories(${PROJECT_NAME}_conversion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_conversion
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_node
  src/fastlio2_to_px4_odometry_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  nav_msgs
  px4_msgs
  rclcpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_conversion
  Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}_conversion ${PROJECT_NAME}_node
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
    test_odometry_conversion
    test/test_odometry_conversion.cpp
  )
  target_link_libraries(test_odometry_conversion
    ${PROJECT_NAME}_conversion
    Eigen3::Eigen
  )

  ament_add_pytest_test(
    test_config_contract
    test/test_config_contract.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_conversion)
ament_export_dependencies(Eigen3)

ament_package()
