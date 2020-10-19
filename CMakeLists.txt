cmake_minimum_required(VERSION 3.15)

project("SCONES")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/MP>")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory("dependencies")
add_subdirectory("src")
add_subdirectory("tests")
add_subdirectory("runner")