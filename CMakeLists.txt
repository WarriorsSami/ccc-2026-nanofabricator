cmake_minimum_required(VERSION 3.20)
project(ccc CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Every level-N/ directory with a CMakeLists.txt becomes its own target.
file(GLOB CCC_LEVELS LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/level-*")
foreach(dir ${CCC_LEVELS})
    if(IS_DIRECTORY ${dir} AND EXISTS ${dir}/CMakeLists.txt)
        add_subdirectory(${dir})
    endif()
endforeach()
