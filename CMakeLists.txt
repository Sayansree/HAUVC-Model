cmake_minimum_required(VERSION 3.1.0)
project(HAUVC-Model)
add_compile_options(-std=c++11)

add_executable(model model.cpp action.h model.h PID-Controller/PID.cpp PID-Controller/PID.h)
