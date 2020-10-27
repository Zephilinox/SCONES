cmake_minimum_required(VERSION 3.15)

find_package(SDL2)

if (NOT SDL2_FOUND)
    message(STATUS "Warning - Skipping Runner. SDL_PATH = ${SDL_PATH}")
    return()
endif()

find_package(OpenGL)

include(FetchContent)

message(STATUS "############ Fetching IMGUI")
FetchContent_Declare(imgui)
if (NOT imgui_POPULATED)
    FetchContent_Populate(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG        455c21df7100a4727dd6e4c8e69249b7de21d24c
        GIT_SHALLOW    TRUE
        GIT_STRATEGY   CHECKOUT
        SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/IMGUI/IMGUI"
    )
    add_library(IMGUI
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/imgui_demo.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_sdl.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
    )
    #target_link_libraries(IMGUI PUBLIC SDL2::Core OpenGL::GL)
    target_include_directories(IMGUI SYSTEM PUBLIC "${imgui_SOURCE_DIR}/")
    target_include_directories(IMGUI SYSTEM PUBLIC "${imgui_SOURCE_DIR}/../")
    target_include_directories(IMGUI SYSTEM PUBLIC "${imgui_SOURCE_DIR}/../backends/")
    set_target_properties(IMGUI PROPERTIES FOLDER dependencies)
    set_target_properties(IMGUI PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_definitions(IMGUI PUBLIC IMGUI_IMPL_OPENGL_LOADER_GL3W GL_GLEXT_PROTOTYPES=1)
    target_link_libraries(IMGUI SDL2::Core gl3w)
    add_dependencies(IMGUI gl3w)
endif()
message(STATUS "############ Done")

message(STATUS "############ Fetching GL3W")
FetchContent_Declare(GL3W
    GIT_REPOSITORY https://github.com/skaslev/gl3w
    GIT_TAG        43eaaad6afb60477a7c755253823785378cc0ae6
    GIT_SHALLOW    TRUE
)
if (NOT GL3W_POPULATED)
    FetchContent_Populate(GL3W)
    add_subdirectory("${gl3w_SOURCE_DIR}" "${gl3w_BINARY_DIR}")
endif()
message(STATUS "############ Done")

project("Runner")

SET(Runner_INCLUDES
    "NotAGame.hpp"

    "Paperbag/RandomCrap.hpp"
    "Paperbag/Window.hpp"
    "Paperbag/Renderer.hpp"
    "Paperbag/GUI.hpp"
)

SET(Runner_SOURCES
    "main.cpp"
    "NotAGame.cpp"

    "Paperbag/Window.cpp"
    "Paperbag/Renderer.cpp"
    "Paperbag/GUI.cpp"
)

add_executable(Runner ${Runner_INCLUDES} ${Runner_SOURCES})

if (WIN32)
    add_custom_target(copy_sdl2_dlls
        COMMAND ${CMAKE_COMMAND} -E copy
                "${SDL2_LIBRARY}/../SDL2.dll"
                "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/SDL2.dll"
    )
    add_dependencies(Runner copy_sdl2_dlls)
endif()

#add_custom_target(copy_runner_resources
#    COMMAND ${CMAKE_COMMAND} -E copy
#            "${CMAKE_CURRENT_SOURCE_DIR}/scone.bmp"
#            "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/scone.bmp"
#)
#add_dependencies(Runner copy_runner_resources)

set_target_properties(Runner PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS ${gl3w_SOURCE_DIR})
target_link_libraries(Runner SCONES SDL2::Main OpenGL::GL IMGUI gl3w)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Runner_INCLUDES} ${Runner_SOURCES})
set_property(TARGET Runner PROPERTY CXX_STANDARD 17)
set_property(TARGET Runner PROPERTY CXX_STANDARD_REQUIRD ON)
set_property(TARGET Runner PROPERTY CXX_EXTENSIONS OFF)
set_property(TARGET Runner PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources")