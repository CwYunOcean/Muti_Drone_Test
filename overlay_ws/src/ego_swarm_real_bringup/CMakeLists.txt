cmake_minimum_required(VERSION 3.8)
project(ego_swarm_real_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_single_real_launch_contract
    test/test_single_real_launch_contract.py
  )
  ament_add_pytest_test(
    test_single_real_rviz_contract
    test/test_single_real_rviz_contract.py
  )
  ament_add_pytest_test(
    test_single_real_script_contract
    test/test_single_real_script_contract.py
  )
endif()

ament_package()
