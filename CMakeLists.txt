cmake_minimum_required(VERSION 3.5)
project(ros_phoenix)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TalonControl.msg"
  "msg/TalonStatus.msg"
)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# List of Phoenix static libraries
set(PHOENIX_LIBRARIES CTRE_Phoenix
                      CTRE_PhoenixCCI
                      CTRE_PhoenixDiagnostics
                      CTRE_PhoenixPlatform_socketcan
                      CTRE_PhoenixCanutils
                      CTRE_PhoenixCore
)

# Create a target for each Phoenix library
foreach(LIBRARY ${PHOENIX_LIBRARIES})
  add_library(${LIBRARY} STATIC IMPORTED)
  set(LIBRARY_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/lib${LIBRARY}.a)
  SET_TARGET_PROPERTIES(${LIBRARY} PROPERTIES IMPORTED_LOCATION ${LIBRARY_LOCATION})
  install (
    FILES ${LIBRARY_LOCATION}
    DESTINATION lib
  )
endforeach()

# Create target for test
add_executable(phoenix_test "src/test.cpp" "src/talon_component.cpp")
ament_target_dependencies(phoenix_test
  "rclcpp"
  "rclcpp_components")
target_link_libraries(phoenix_test ${PHOENIX_LIBRARIES} ros_phoenix__rosidl_typesupport_cpp)

# Create target for phoenix_container
add_executable(phoenix_container "src/phoenix_container.cpp")
ament_target_dependencies(phoenix_container
  "rclcpp"
  "rclcpp_components")
target_link_libraries(phoenix_container ${PHOENIX_LIBRARIES})

# Create target for TalonComponent
add_library(talon_component SHARED
   src/talon_component.cpp)
ament_target_dependencies(talon_component
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "std_msgs")
rclcpp_components_register_nodes(talon_component "ros_phoenix::TalonComponent")
target_link_libraries(talon_component ${PHOENIX_LIBRARIES} ros_phoenix__rosidl_typesupport_cpp)

# Install Components
install(TARGETS phoenix_container talon_component phoenix_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()