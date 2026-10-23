cmake_minimum_required(VERSION 3.20)
project(level-3 CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(level-3 main.cpp)
target_include_directories(level-3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(level-3 PRIVATE CCC_LEVEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
