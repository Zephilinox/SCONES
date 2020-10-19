cmake_minimum_required(VERSION 3.15)

project("Tests")

add_executable(Tests
    "main.cpp"
    "TestCPU.cpp"
)

set_property(TARGET Tests PROPERTY CXX_STANDARD 17)
set_property(TARGET Tests PROPERTY CXX_EXTENSIONS OFF)

target_link_libraries(Tests PUBLIC SCONES doctest)
target_include_directories(Tests PUBLIC ${SCONES_BINARY_DIR}/dependencies/doctest-src/doctest)