cmake_minimum_required(VERSION 3.15)

find_package(SDL2)

if (NOT SDL2_FOUND)
    message(STATUS "Warning - Skipping Runner. SDL_PATH = ${SDL_PATH}")
    return()
endif()

project("Runner")

SET(Runner_INCLUDES

)

SET(Runner_SOURCES
    "main.cpp"
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

add_custom_target(copy_runner_resources
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_SOURCE_DIR}/scone.bmp"
            "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/scone.bmp"
)
add_dependencies(Runner copy_runner_resources)

set_target_properties(Runner PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(Runner SCONES SDL2::Main)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Runner_INCLUDES} ${Runner_SOURCES})
set_property(TARGET SCONES PROPERTY CXX_STANDARD 17)
set_property(TARGET SCONES PROPERTY CXX_EXTENSIONS OFF)