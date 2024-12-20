cmake_minimum_required(VERSION 3.11)
project(nbody)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)

set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET)
if (NOT raylib_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED)
    set(FETCHCONTENT_QUIET NO)
    FetchContent_MakeAvailable(raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  endif()
endif()

add_compile_options(
  -Wall -Wpedantic -Wextra "$<$<CONFIG:DEBUG>:-O0;-g3;-ggdb>"
)

add_executable(${PROJECT_NAME} main.cpp)
set(raylib_VERBOSE 1)
target_link_libraries(${PROJECT_NAME} PRIVATE raylib "-lstdc++exp")
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${raylib_INCLUDE_DIRS})
