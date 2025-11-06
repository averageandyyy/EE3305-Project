cmake_minimum_required(VERSION 3.8)
project(ee3305_map_loader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set (DEPENDENCIES
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "nav_msgs"
)

######## MAP_LOADER ########
add_executable(map_loader src/map_loader.cpp)
target_include_directories(map_loader PUBLIC include)
ament_target_dependencies(map_loader ${DEPENDENCIES})
target_compile_features(map_loader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS map_loader DESTINATION lib/${PROJECT_NAME})

ament_package()
