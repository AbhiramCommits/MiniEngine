# Optional windowed sandbox front-end. Only configured when
# MINIENGINE_BUILD_APP=ON (default locally, OFF in CI).

include(FetchContent)

# --- GLFW ------------------------------------------------------------------
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glfw)

# --- Dear ImGui (ships no CMakeLists, so we build it ourselves) -------------
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.9
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(minieng_imgui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)
target_include_directories(minieng_imgui PUBLIC
    "${imgui_SOURCE_DIR}"
    "${imgui_SOURCE_DIR}/backends"
)
target_link_libraries(minieng_imgui PUBLIC glfw)
target_compile_definitions(minieng_imgui PRIVATE GL_SILENCE_DEPRECATION)

find_package(OpenGL REQUIRED)

add_executable(minieng_sandbox
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
)
target_link_libraries(minieng_sandbox PRIVATE
    minieng::core
    minieng_imgui
    glfw
    OpenGL::GL
    minieng_warnings
)
target_compile_definitions(minieng_sandbox PRIVATE
    MINIENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    GL_SILENCE_DEPRECATION
)
