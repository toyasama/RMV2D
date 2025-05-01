cmake_minimum_required(VERSION 3.8)
project(rmv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set (SRV_FILES
  "srv/UpdateParams.srv"
)

set(MSG_FILES
  "msg/ParamsRequest.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces
)

install(DIRECTORY srv msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
