cmake_minimum_required(VERSION 3.8)
project(ee3305_bringup)

# g++ warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# dependencies
find_package(ament_cmake REQUIRED)

# for folders to appear in the install directory
install (
  DIRECTORY launch rviz params maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
