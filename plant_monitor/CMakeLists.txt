cmake_minimum_required(VERSION 3.8)
project(plant_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)



ament_python_install_package(scripts)


install(
  PROGRAMS  scripts/check_ph.py
            scripts/light_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch scripts config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()