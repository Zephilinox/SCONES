cmake_minimum_required(VERSION 3.15)

project("Runner")

SET(Runner_INCLUDES

)

SET(Runner_SOURCES
    "main.cpp"
)

add_executable(Runner ${Runner_INCLUDES} ${Runner_SOURCES})
target_link_libraries(Runner SCONES)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Runner_INCLUDES} ${Runner_SOURCES})
set_property(TARGET SCONES PROPERTY CXX_STANDARD 17)
set_property(TARGET SCONES PROPERTY CXX_EXTENSIONS OFF)