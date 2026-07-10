cmake_minimum_required(VERSION 3.16)
project(iseda_lut_runner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(generate_lut_table
  generate_lut_table.cpp
)
set_target_properties(
  generate_lut_table
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/build"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/build"
)

target_include_directories(generate_lut_table PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
