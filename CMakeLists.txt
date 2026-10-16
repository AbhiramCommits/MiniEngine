cmake_minimum_required(VERSION 3.16)

project(MiniEngine
    VERSION 0.1.0
    DESCRIPTION "Small but real C++17 gameplay sandbox engine"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Place binaries in a predictable spot so tools/bench and CI scripts can find
# them without hunting through generator-specific directories.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

option(MINIENGINE_BUILD_APP    "Build the GLFW/Dear ImGui sandbox app" ON)
option(MINIENGINE_BUILD_TESTS  "Build the GoogleTest suite"          ON)

# Single place that owns the project's warning policy. Applied to every
# first-party target; third-party code (gtest, glfw, imgui) is left alone.
add_library(minieng_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(minieng_warnings INTERFACE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(minieng_warnings INTERFACE /W4)
endif()

enable_testing()

add_subdirectory(engine)

if(MINIENGINE_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(MINIENGINE_BUILD_APP)
    add_subdirectory(app)
endif()

message(STATUS "MiniEngine: build_app=${MINIENGINE_BUILD_APP} build_tests=${MINIENGINE_BUILD_TESTS} type=${CMAKE_BUILD_TYPE}")
