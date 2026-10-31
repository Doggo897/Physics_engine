cmake_minimum_required(VERSION 3.26)
project(untitled C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(MyProject
        helper.c
        "Simulation/main.cpp"
        Math/polygon.c
        Math/SAT.c
)
target_include_directories(MyProject PRIVATE
        ${PROJECT_SOURCE_DIR}/Math
)