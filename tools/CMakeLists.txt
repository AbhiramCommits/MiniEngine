# Headless tools. minieng_bench has no window/rendering dependency, so it runs
# in CI and produces the numbers quoted in the README.

add_executable(minieng_bench "${CMAKE_CURRENT_SOURCE_DIR}/bench.cpp")
target_link_libraries(minieng_bench PRIVATE minieng::core minieng_warnings)
