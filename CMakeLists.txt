#
# This is based on orocos sample cmakelists, modified slightly
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.8.3
#
cmake_minimum_required(VERSION 2.8.3)
 
#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(orocos_ros_joint_state_publisher)
 
# current source and include:
include_directories(${PROJECT_SOURCE_DIR}/include)

 
# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(rtt-core-extensions REQUIRED)
find_package(urdf REQUIRED COMPONENTS)
find_package(srdfdom REQUIRED COMPONENTS)

include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})


# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
 
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#
 
 
# Creates a component library libexample-<target>.so
# and installs in the directory lib/orocos/example/
#
# orocos_component(${CMAKE_PROJECT_NAME} simple-rtt-component.hpp simple-rtt-component.cpp) # ...you may add multiple source files
orocos_component(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/orocos_ros_joint_state_publisher.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.
 
#
# Building a typekit (recommended):
#
# Creates a typekit library libexample-types-<target>.so
# and installs in the directory lib/orocos/example/types/
#
#orocos_typegen_headers(example-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !
 
#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.
 
 
#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libexample-service-<target>.so or libexample-plugin-<target>.so
# and installs in the directory lib/orocos/example/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_service(example-service example-service.cpp) # ...only one service per library !
#orocos_plugin(example-plugin example-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin/orocos_service statements.

# target_link_libraries()
target_link_libraries(${CMAKE_PROJECT_NAME} ${RST-RT_LIBRARIES} ${RTT-CORE-EXTENSIONS_LIBRARIES} ${urdf_LIBRARIES})
 
#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/example/ directory
#
# orocos_install_headers( example-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
 
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
