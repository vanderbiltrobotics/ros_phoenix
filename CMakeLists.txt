cmake_minimum_required(VERSION 2.8.3)
project(ros_phoenix)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# Set CTRE_ARCH
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    set(CTRE_ARCH "raspberry")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    set(CTRE_ARCH "jetsontx")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "x86_64-linux-gnu")
    set(CTRE_ARCH "x86-64")
else()
    message(FATAL_ERROR "Unsupported architecture for CTRE Phoenix: ${CMAKE_LIBRARY_ARCHITECTURE}")
endif()

set(CTRE_VERSION "5.19.4")

# Macro for downloading CTRE Libraries
macro(add_ctre_library LIB_NAME URL_NAME)
    SET(LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/ctre/${URL_NAME}")
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
        COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_CURRENT_BINARY_DIR}/ctre/linux/${CTRE_ARCH}/static/lib${LIB_NAME}.a
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${FILE_DEST}
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${HEADER_DEST}
        WORKING_DIRECTORY ${LIB_DIR}
        COMMENT "Unpacking ${LIB_NAME}"
        VERBATIM)

    # Add library
    add_library(${LIB_NAME} STATIC IMPORTED)
    add_dependencies(${LIB_NAME} ${LIB_NAME}-unzip)
    set_target_properties(${LIB_NAME} PROPERTIES
        IMPORTED_LOCATION ${LIB_DEST}
        INTERFACE_INCLUDE_DIRECTORIES ${LIB_DIR}
    )

    # Mark files for installation by catkin
    install (
        FILES ${LIB_DEST}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    install (
        DIRECTORY ${LIB_DIR}/ctre
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
endmacro()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    geometry_msgs
    dynamic_reconfigure
    roslaunch
    message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    MotorControl.msg
    MotorStatus.msg
    ArmVelocities.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

## add dynamic reconfigure api
generate_dynamic_reconfigure_options(
    ${PROJECT_SOURCE_DIR}/cfg/Talon.cfg
)

## Generate launch files
roslaunch_add_file_check(${PROJECT_SOURCE_DIR}/launch/example.launch)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
set(PHOENIX_LIBRARIES
    CTRE_Phoenix
    CTRE_PhoenixCCI
    CTRE_PhoenixDiagnostics
    CTRE_PhoenixPlatform_socketcan
    CTRE_PhoenixCanutils
    CTRE_PhoenixCore
)

catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    LIBRARIES ${PHOENIX_LIBRARIES}
    CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure roslaunch message_runtime
)

# Download the CTRE libraries
add_ctre_library(CTRE_Phoenix                   api-cpp)
add_ctre_library(CTRE_PhoenixCCI                cci)
add_ctre_library(CTRE_PhoenixDiagnostics        diagnostics)
add_ctre_library(CTRE_PhoenixPlatform_socketcan platform-socketcan)
add_ctre_library(CTRE_PhoenixCanutils           canutils)
add_ctre_library(CTRE_PhoenixCore               core)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    SYSTEM ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/phoenix_node.cpp)
#add_executable(phoenix_node ${PROJECT_SOURCE_DIR}/example.cpp)
add_executable(ros_phoenix_cmd_example ${PROJECT_SOURCE_DIR}/src/CmdExmaple.cpp)
add_dependencies(ros_phoenix_cmd_example ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_phoenix_fd_teleop ${PROJECT_SOURCE_DIR}/src/FDTeleop.cpp)
add_dependencies(ros_phoenix_fd_teleop ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_phoenix_arm_command ${PROJECT_SOURCE_DIR}/src/ArmCommand.cpp)
add_dependencies(ros_phoenix_arm_command ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_phoenix_deposition_command ${PROJECT_SOURCE_DIR}/src/DepositionCommand.cpp)
add_dependencies(ros_phoenix_deposition_command ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_phoenix_node
    ${PROJECT_SOURCE_DIR}/src/TalonNode.cpp
    ${PROJECT_SOURCE_DIR}/src/MotorControl.cpp
)
add_dependencies(ros_phoenix_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fake_ros_phoenix_node
    ${PROJECT_SOURCE_DIR}/src/FakeTalonNode.cpp
    ${PROJECT_SOURCE_DIR}/src/FakeMotorControl.cpp
)
add_dependencies(fake_ros_phoenix_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_phoenix_cmd_example
    ${catkin_LIBRARIES}
    Threads::Threads
    ${PHOENIX_LIBRARIES}
)
target_link_libraries(ros_phoenix_fd_teleop
    ${catkin_LIBRARIES}
    Threads::Threads
    ${PHOENIX_LIBRARIES}
)
target_link_libraries(ros_phoenix_arm_command
    ${catkin_LIBRARIES}
    Threads::Threads
    ${PHOENIX_LIBRARIES}
)
target_link_libraries(ros_phoenix_deposition_command
    ${catkin_LIBRARIES}
    Threads::Threads
    ${PHOENIX_LIBRARIES}
)
target_link_libraries(ros_phoenix_node
    ${catkin_LIBRARIES}
    Threads::Threads
    ${PHOENIX_LIBRARIES}
)
target_link_libraries(fake_ros_phoenix_node
    ${catkin_LIBRARIES}
    Threads::Threads
    ${PHOENIX_LIBRARIES}
)

#############
## Install ##
#############

## Install python scripts
catkin_install_python(PROGRAMS
    scripts/box_testing.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install node executables
install(TARGETS fake_ros_phoenix_node ros_phoenix_node ros_phoenix_fd_teleop ros_phoenix_cmd_example ros_phoenix_arm_command ros_phoenix_deposition_command
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/ros_phoenix/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
## Mark launch files for installation
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
)
