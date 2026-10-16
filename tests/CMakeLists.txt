include(FetchContent)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(googletest)

# Every tests/test_*.cpp becomes its own ctest entry, which keeps failures easy
# to attribute and lets CI build a single suite if needed.
file(GLOB MINIENGINE_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")

foreach(test_src ${MINIENGINE_TEST_SOURCES})
    get_filename_component(test_name "${test_src}" NAME_WE)
    add_executable(${test_name} "${test_src}")
    target_link_libraries(${test_name} PRIVATE minieng::core minieng_warnings GTest::gtest_main)
    target_compile_definitions(${test_name} PRIVATE
        MINIENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

list(LENGTH MINIENGINE_TEST_SOURCES _n_test_sources)
message(STATUS "MiniEngine: registered ${_n_test_sources} test target(s)")
