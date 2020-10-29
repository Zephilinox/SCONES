cmake_minimum_required(VERSION 3.15)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/dependencies/find_sdl2-src")
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

project("SCONES")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/MP>")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory("dependencies")
add_subdirectory("src")
add_subdirectory("tests")
add_subdirectory("runner")

enable_testing()
add_test(Tests Release ${CMAKE_CURRENT_SOURCE_DIR}/resources)