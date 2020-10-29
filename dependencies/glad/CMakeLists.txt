cmake_minimum_required(VERSION 3.15)
project(GLAD)

add_library(GLAD include/KHR/khrplatform.h include/glad/glad.h src/glad.c)
target_include_directories(GLAD PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<INSTALL_INTERFACE:include>)
set_target_properties(GLAD PROPERTIES FOLDER dependencies)