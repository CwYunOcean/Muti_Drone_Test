cmake_minimum_required(VERSION 3.8)
project(gvf_path_following_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GVFReference.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_message_contract
    test/test_message_contract.py
  )
endif()

ament_export_dependencies(rosidl_default_runtime geometry_msgs std_msgs)
ament_package()
