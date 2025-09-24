cmake_minimum_required(VERSION 3.8)
project(regolife_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)



ament_python_install_package(scripts)


install(
  PROGRAMS  scripts/aruco.py
            scripts/detect_plants.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch scripts config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()