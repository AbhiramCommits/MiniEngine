# minieng_core: the headless simulation library. No windowing, no rendering,
# no third-party dependencies. This is what CI builds and tests.

file(GLOB_RECURSE MINIENGINE_CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(minieng_core STATIC ${MINIENGINE_CORE_SOURCES})

target_include_directories(minieng_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(minieng_core PRIVATE minieng_warnings)

# Convenience alias used by app/tools/tests.
add_library(minieng::core ALIAS minieng_core)
