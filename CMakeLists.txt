cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

## Detect system architecture and save it to $arch
EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE arch)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        geometry_msgs
        dynamic_reconfigure
        roslaunch
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#)

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
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils
        CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure roslaunch
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        SYSTEM ${catkin_INCLUDE_DIRS}
)

## Declare libraries to be imported
ADD_LIBRARY(CTRE_Phoenix STATIC IMPORTED)
ADD_LIBRARY(CTRE_PhoenixCCI STATIC IMPORTED)
ADD_LIBRARY(CTRE_PhoenixPlatformLinuxSocketCan STATIC IMPORTED)
ADD_LIBRARY(CTRE_PhoenixCanutils STATIC IMPORTED)

## Import appropriate libraries for architecture
if (${arch} STREQUAL "x86_64" OR ${arch} STREQUAL "armv7l" OR ${arch} STREQUAL "aarch64")
    SET_TARGET_PROPERTIES(CTRE_Phoenix PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_Phoenix.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixCCI PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_PhoenixCCI.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixPlatformLinuxSocketCan PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixCanutils PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_PhoenixCanutils.a)
else ()
    message(FATAL_ERROR "Unrecognized architecture: ${arch}")
endif ()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/phoenix_node.cpp)
#add_executable(phoenix_node ${PROJECT_SOURCE_DIR}/example.cpp)
add_executable(motor_control_cmd_example ${PROJECT_SOURCE_DIR}/src/CmdExmaple.cpp)
add_dependencies(motor_control_cmd_example ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motor_control_node
        ${PROJECT_SOURCE_DIR}/src/TalonNode.cpp
        ${PROJECT_SOURCE_DIR}/src/MotorControl.cpp
        )
add_dependencies(motor_control_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(motor_control_cmd_example
        ${catkin_LIBRARIES}
        Threads::Threads
        CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils
        )
target_link_libraries(motor_control_node
        ${catkin_LIBRARIES}
        Threads::Threads
        CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils
        )

#############
## Install ##
#############

## Install runtime libraries
install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_Phoenix.a
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_PhoenixCanutils.a
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_PhoenixCCI.a
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/${arch}/libCTRE_PhoenixPlatformLinuxSocketCan.a
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
## Install node executables
install(TARGETS motor_control_node motor_control_cmd_example
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
## Mark launch files for installation
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
        )