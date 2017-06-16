cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
# Give a name to the project.
project(pepper_navigation)

#set(CMAKE_VERBOSE_MAKEFILE ON)

# This include enable you to use qibuild framework
find_package(qibuild)

message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(src)
add_subdirectory(src)
