
cmake_minimum_required(VERSION 3.21)
project(insect-ai)
set(CMAKE_CXX_STANDARD 11)
add_subdirectory(external/raylib)

set(src
    src/actuator.cpp
    src/Agent.cpp
    src/Clock.cpp
    src/Clock.h
    src/demo.cpp
    src/demo.h
    src/function.cpp
    src/hodographs.c
    src/hodographs.h
    src/InsectAI.cpp
    src/InsectAI.h
    src/InsectAI_Actuator.h
    src/InsectAI_Agent.h
    src/InsectAI_Engine.h
    src/InsectAI_Sensor.h
    src/InsectAI_Vehicle.h
    src/light.cpp
    src/lq.c
    src/lq.h
    src/main.cpp
    src/NearestNeighbours.cpp
    src/NearestNeighbours.h
    src/PMath.cpp
    src/PMath.h
    src/raygui.h
    src/Resource.h
    src/sensor.cpp
    src/vehicle.cpp
)

add_executable(insect-ai ${src})
target_link_libraries(insect-ai raylib)
