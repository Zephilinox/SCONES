cmake_minimum_required(VERSION 3.15)

project("SCONES")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/MP>")
add_subdirectory("dependencies")
add_subdirectory("tests")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
SET(SCONES_INCLUDES
    "src/NES/APU.hpp"
    "src/NES/Bus.hpp"
    "src/NES/Cartridge.hpp"
    "src/NES/Controller.hpp"
    "src/NES/CPU.hpp"
    "src/NES/Mapper.hpp"
    "src/NES/Mapper0.hpp"
    "src/NES/PPU.hpp"
)

SET(SCONES_SOURCES
    "src/NES/APU.cpp"
    "src/NES/Bus.cpp"
    "src/NES/Cartridge.cpp"
    "src/NES/Controller.cpp"
    "src/NES/CPU.cpp"
    "src/NES/Mapper.cpp"
    "src/NES/Mapper0.cpp"
    "src/NES/PPU.cpp"
    "src/main.cpp"
)

add_executable(SCONES ${SCONES_INCLUDES} ${SCONES_SOURCES})
target_link_libraries(SCONES spdlog)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SCONES_INCLUDES} ${SCONES_SOURCES})
set_property(TARGET SCONES PROPERTY CXX_STANDARD 17)
set_property(TARGET SCONES PROPERTY CXX_EXTENSIONS OFF CXX_STANDARD_REQUIRED ON)