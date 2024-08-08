cmake_minimum_required(VERSION 3.8)
project(footprint_observer)

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
  rosidl_default_generators
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

### BOOST ###
## first, install packages with `sudo apt install libboost-all-dev`
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetFootprint.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
 )

# Add executable
add_executable(footprint_observer_node src/footprint_observer.cpp)
target_link_libraries(footprint_observer_node ${Boost_LIBRARIES})
target_include_directories(
  footprint_observer_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
ament_target_dependencies(
  footprint_observer_node PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  TARGETS
  footprint_observer_node
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
  scan_unifier_node
)

ament_package()

