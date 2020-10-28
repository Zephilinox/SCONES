cmake_minimum_required(VERSION 3.15)
project(GLAD)

add_library(GLAD include/KHR/khrplatform.h include/glad/glad.h src/glad.c)
target_include_directories(GLAD PUBLIC include/)
set_target_properties(GLAD PROPERTIES FOLDER dependencies)