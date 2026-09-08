cmake_minimum_required(VERSION 3.16)
project(OmniGUI LANGUAGES C CXX)

# ============================================================
#  Global Settings
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default ke Debug kalau user tidak set apa-apa
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# ============================================================
#  Options
# ============================================================
option(USE_GLES2 "Build with OpenGL ES2 (Raspberry Pi)" ON)

# ============================================================
#  Dependencies
# ============================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# ============================================================
#  Executable
# ============================================================
add_executable(omni_gui
    main.cpp

    imgui_docking/imgui.cpp
    imgui_docking/imgui_draw.cpp
    imgui_docking/imgui_tables.cpp
    imgui_docking/imgui_widgets.cpp
    # imgui_docking/imgui_demo.cpp

    imgui_docking/backends/imgui_impl_sdl2.cpp
    imgui_docking/backends/imgui_impl_opengl3.cpp
)

# ============================================================
#  Include Directories
# ============================================================
target_include_directories(omni_gui PRIVATE
    ${SDL2_INCLUDE_DIRS}
    imgui_docking
    imgui_docking/backends
)

# ============================================================
#  Compile Flags (PER CONFIG)
# ============================================================
target_compile_options(omni_gui PRIVATE
    ${SDL2_CFLAGS_OTHER}

    # Debug flags
    $<$<CONFIG:Debug>:
        -g3
        -O0
        -fno-omit-frame-pointer
        -Wall
        -Wextra
    >

    # Release flags
    $<$<CONFIG:Release>:
        -O3
        -DNDEBUG
    >
)

# ============================================================
#  Link Libraries
# ============================================================
target_link_libraries(omni_gui PRIVATE
    ${SDL2_LIBRARIES}
)

# ============================================================
#  OpenGL / GLES Config
# ============================================================
if(USE_GLES2)
    target_compile_definitions(omni_gui PRIVATE
        USE_GLES2=1
        IMGUI_IMPL_OPENGL_ES2=1
    )

    target_link_libraries(omni_gui PRIVATE
        GLESv2
        EGL
    )
endif()

# ============================================================
#  Debug Info Print
# ============================================================
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Using GLES2 : ${USE_GLES2}")