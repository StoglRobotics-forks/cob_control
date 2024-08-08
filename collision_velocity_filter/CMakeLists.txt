cmake_minimum_required(VERSION 3.8)
project(collision_velocity_filter)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra 
  -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow 
  -Wno-float-conversion -Wno-sign-compare)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

### BOOST ###
## first, install packages with `sudo apt install libboost-all-dev`
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Add executable
add_executable(collision_velocity_filter_node src/collision_velocity_filter.cpp)
target_link_libraries(collision_velocity_filter_node ${Boost_LIBRARIES})
target_include_directories(collision_velocity_filter_node PRIVATE
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(collision_velocity_filter_node ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(
  TARGETS
  collision_velocity_filter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(
  collision_velocity_filter_node
)

ament_package()

