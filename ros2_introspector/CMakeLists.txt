cmake_minimum_required(VERSION 3.8)
project(ros2_introspector)

# Get the path to the workspace directory
get_filename_component(WORKSPACE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# the ROS2 node (=executable) name 
set(NODE_NAME ros2_introspector_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Set output from clang-tidy checks at compile time
  set(CMAKE_CXX_CLANG_TIDY
      clang-tidy
      -checks=-*,cppcoreguidelines-*
)

endif()

# Create config file <compile_commands.json>
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
# Specify version requirements for yaml-cpp and spdlog
find_package(yaml-cpp 0.8.0 REQUIRED)
find_package(spdlog 1.12.0 REQUIRED)

# Include the include directory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${WORKSPACE_DIR}
)

add_executable(
  ${NODE_NAME}  
  src/main.cpp
  src/introspection_helper.cpp
  src/type_description_client.cpp
  src/asyncapi_generator.cpp
)

ament_target_dependencies(
  ${NODE_NAME} 
  rclcpp
  )

# Link include directories to the target
target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${WORKSPACE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${NODE_NAME}
  yaml-cpp
  spdlog::spdlog
)

install(TARGETS ${NODE_NAME} 
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install utils headers
install(DIRECTORY ${WORKSPACE_DIR}/utils
  DESTINATION include/utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
