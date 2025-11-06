cmake_minimum_required(VERSION 3.8)
project(ee3305_cpp)

# g++ warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set (DEPENDENCIES
  "rclcpp"
  "rcutils"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
)

# behavior node
add_executable(behavior src/behavior.cpp)
target_include_directories(behavior PUBLIC include)
ament_target_dependencies(behavior ${DEPENDENCIES})
target_compile_features(behavior PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS behavior DESTINATION lib/${PROJECT_NAME})

# controller node
add_executable(controller src/controller.cpp)
target_include_directories(controller PUBLIC include)
ament_target_dependencies(controller ${DEPENDENCIES})
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS controller DESTINATION lib/${PROJECT_NAME})

# planner node
add_executable(planner src/planner.cpp)
target_include_directories(planner PUBLIC include)
ament_target_dependencies(planner ${DEPENDENCIES})
target_compile_features(planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS planner DESTINATION lib/${PROJECT_NAME})

ament_package()
