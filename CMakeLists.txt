cmake_minimum_required(VERSION 3.15)

project("SCONES")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/MP>")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_executable(SCONES ${SCONES_INCLUDES} ${SCONES_SOURCES})
target_link_libraries(SCONES spdlog)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SCONES_INCLUDES} ${SCONES_SOURCES})
set_property(TARGET SCONES PROPERTY CXX_STANDARD 17)
set_property(TARGET SCONES PROPERTY CXX_EXTENSIONS OFF CXX_STANDARD_REQUIRED ON)

add_subdirectory("dependencies")
add_subdirectory("src")
add_subdirectory("tests")
add_subdirectory("runner")