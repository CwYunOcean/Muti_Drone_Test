cmake_minimum_required(VERSION 3.8)
project(fast_livo2_mid360_bringup)

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
    test_mid360_lio_config_contract
    test/test_mid360_lio_config_contract.py
  )
  ament_add_pytest_test(
    test_mid360_lio_launch_contract
    test/test_mid360_lio_launch_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_config_contract
    test/test_mid360_livo_d435i_config_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_launch_contract
    test/test_mid360_livo_d435i_launch_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_script_contract
    test/test_mid360_livo_d435i_script_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_sync_experiment_launch_contract
    test/test_mid360_livo_d435i_sync_experiment_launch_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_sync_script_contract
    test/test_mid360_livo_d435i_sync_script_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_rviz_contract
    test/test_mid360_livo_d435i_rviz_contract.py
  )
  ament_add_pytest_test(
    test_record_retail_street_style_bag_script_contract
    test/test_record_retail_street_style_bag_script_contract.py
  )
  ament_add_pytest_test(
    test_mid360_livo_d435i_config_types_contract
    test/test_mid360_livo_d435i_config_types_contract.py
  )
endif()

ament_package()
