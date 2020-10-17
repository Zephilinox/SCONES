cmake_minimum_required (VERSION 3.15)

project("SCONES")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/MP>")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

SET(SCONES_INCLUDES
    ""
)

SET(SCONES_SOURCES
    "src/main.cpp"
)

add_executable(SCONES ${SCONES_SOURCES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SCONES_SOURCES})