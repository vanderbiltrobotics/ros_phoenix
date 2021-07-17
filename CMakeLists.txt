cmake_minimum_required(VERSION 3.5)
project(ros_phoenix)

# Set CTRE_ARCH
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    set(CTRE_ARCH "raspbian")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    set(CTRE_ARCH "aarch64bionic")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "x86_64-linux-gnu")
    set(CTRE_ARCH "x86-64")
else()
    message(FATAL_ERROR "Unsupported architecture for CTRE Phoenix: ${CMAKE_LIBRARY_ARCHITECTURE}")
endif()

set(CTRE_VERSION "5.19.4")

# Macro for downloading CTRE Libraries
macro(add_ctre_library LIB_NAME URL_NAME)
    SET(LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/ctre/${CTRE_VERSION}/${URL_NAME}")
    SET(LIB_DEST "${LIB_DIR}/linux/${CTRE_ARCH}/static/lib${LIB_NAME}.a")
    file(MAKE_DIRECTORY ${LIB_DIR})
    file(MAKE_DIRECTORY ${LIB_DIR}/ctre)

    # Download the library
    set(FILE_NAME "${URL_NAME}-${CTRE_VERSION}-linux${CTRE_ARCH}static.zip")
    set(FILE_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${FILE_NAME}")
    set(FILE_DEST "${LIB_DIR}/${FILE_NAME}")
    if(NOT EXISTS ${FILE_DEST})
        message(STATUS "Downloading ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${FILE_URL} ${FILE_DEST})
    endif()

    # Download the headers
    SET(HEADER_NAME "${URL_NAME}-${CTRE_VERSION}-headers.zip")
    set(HEADER_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${HEADER_NAME}")
    set(HEADER_DEST "${LIB_DIR}/${HEADER_NAME}")
    if(NOT EXISTS ${HEADER_DEST})
        message(STATUS "Downloading headers for ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${HEADER_URL} ${HEADER_DEST})
    endif()

    # Create a target for extracing the zip files
    add_custom_target(${LIB_NAME}-unzip DEPENDS ${LIB_DEST})
    add_custom_command(OUTPUT ${LIB_DEST} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${FILE_DEST}
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${HEADER_DEST}
        WORKING_DIRECTORY ${LIB_DIR}
        COMMENT "Unpacking ${LIB_NAME}"
        VERBATIM
    )

    # Add library
    add_library(${LIB_NAME} STATIC IMPORTED)
    add_dependencies(${LIB_NAME} ${LIB_NAME}-unzip)
    set_target_properties(${LIB_NAME} PROPERTIES
        IMPORTED_LOCATION ${LIB_DEST}
        INTERFACE_INCLUDE_DIRECTORIES ${LIB_DIR}
    )
endmacro()

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
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorControl.msg"
  "msg/MotorStatus.msg"
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# List of Phoenix static libraries
set(PHOENIX_LIBRARIES CTRE_Phoenix
                      CTRE_PhoenixCCI
                      CTRE_PhoenixDiagnostics
                      CTRE_PhoenixPlatform_socketcan
                      CTRE_PhoenixCanutils
                      CTRE_PhoenixCore
)

# Download the CTRE libraries
add_ctre_library(CTRE_Phoenix                   api-cpp)
add_ctre_library(CTRE_PhoenixCCI                cci)
add_ctre_library(CTRE_PhoenixDiagnostics        diagnostics)
add_ctre_library(CTRE_PhoenixPlatform_socketcan platform-socketcan)
add_ctre_library(CTRE_PhoenixCanutils           canutils)
add_ctre_library(CTRE_PhoenixCore               core)

# Create shared library with components and CTRE libraries
add_library(phoenix_components SHARED
   "src/phoenix_components.cpp"
   "src/phoenix_manager.cpp")
ament_target_dependencies(phoenix_components
  "rclcpp"
  "rclcpp_components"
  "rcutils")
target_link_libraries(phoenix_components
  ${PHOENIX_LIBRARIES}
  Threads::Threads
  ros_phoenix__rosidl_typesupport_cpp)

# Register components
rclcpp_components_register_nodes(phoenix_components "ros_phoenix::VictorSPX")
rclcpp_components_register_nodes(phoenix_components "ros_phoenix::TalonSRX")
rclcpp_components_register_nodes(phoenix_components "ros_phoenix::TalonFX")

# Create executable for phoenix_container which links phoenix_components
add_executable(phoenix_container "src/phoenix_container.cpp")
ament_target_dependencies(phoenix_container
  "rclcpp"
  "rclcpp_components")
target_link_libraries(phoenix_container phoenix_components)

# Install Components
install(TARGETS phoenix_components phoenix_container
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
