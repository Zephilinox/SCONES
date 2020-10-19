cmake_minimum_required(VERSION 3.15)

SET(SCONES_INCLUDES
    "SCONES/APU.hpp"
    "SCONES/Bus.hpp"
    "SCONES/Cartridge.hpp"
    "SCONES/Controller.hpp"
    "SCONES/CPU.hpp"
    "SCONES/Mapper.hpp"
    "SCONES/Mapper0.hpp"
    "SCONES/PPU.hpp"
    "SCONES/NES.hpp"
    "SCONES/Framebuffer.hpp"
)

SET(SCONES_SOURCES
    "SCONES/APU.cpp"
    "SCONES/Bus.cpp"
    "SCONES/Cartridge.cpp"
    "SCONES/Controller.cpp"
    "SCONES/CPU.cpp"
    "SCONES/Mapper.cpp"
    "SCONES/Mapper0.cpp"
    "SCONES/PPU.cpp"
    "SCONES/NES.cpp"
    "SCONES/Framebuffer.cpp"
)

add_library(SCONES ${SCONES_INCLUDES} ${SCONES_SOURCES})
target_include_directories(SCONES PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(SCONES spdlog)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SCONES_INCLUDES} ${SCONES_SOURCES})
set_property(TARGET SCONES PROPERTY CXX_STANDARD 17)
set_property(TARGET SCONES PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET SCONES PROPERTY CXX_EXTENSIONS OFF)