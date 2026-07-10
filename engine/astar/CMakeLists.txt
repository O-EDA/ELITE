cmake_minimum_required(VERSION 3.16)
project(elite_astar_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ASTAR_COPY_SOURCES
  src/Astar.cpp
  src/GR.cpp
  src/Grid.cpp
)

add_library(elite_astar_core STATIC ${ASTAR_COPY_SOURCES})
target_include_directories(elite_astar_core PUBLIC src)
set_target_properties(elite_astar_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(MSVC)
  target_compile_options(elite_astar_core PRIVATE /W3)
else()
  target_compile_options(elite_astar_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
