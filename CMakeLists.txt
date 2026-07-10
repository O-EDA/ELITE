cmake_minimum_required(VERSION 3.18)
project(elite_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(PYBIND11_FINDPYTHON ON)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

set(ELITE_LUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/engine/lut/source")

add_subdirectory(engine/astar EXCLUDE_FROM_ALL)

pybind11_add_module(_elite_cpp engine/native/bindings.cpp)
target_link_libraries(_elite_cpp PRIVATE elite_astar_core)
target_include_directories(_elite_cpp PRIVATE
  ${ELITE_LUT_DIR}/include
)

if(MSVC)
  target_compile_options(_elite_cpp PRIVATE /W3 /bigobj)
else()
  target_compile_options(_elite_cpp PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

install(TARGETS _elite_cpp LIBRARY DESTINATION . RUNTIME DESTINATION .)
