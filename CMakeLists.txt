cmake_minimum_required(VERSION 3.16)
project(guided_recovery CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(GR_WARNINGS -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter)

# ---- Flight core (the same sources the ESP32 target compiles)
add_library(grcore STATIC
  firmware/core/src/estimators.cpp
  firmware/core/src/phase_detector.cpp
  firmware/core/src/guidance.cpp
  firmware/core/src/actuation.cpp
  firmware/core/src/flight_core.cpp
  firmware/core/src/ubx.cpp
  firmware/core/src/hil.cpp
  firmware/core/src/redundancy.cpp
)
target_include_directories(grcore PUBLIC firmware/core/include)
target_compile_options(grcore PRIVATE ${GR_WARNINGS} -Werror)

# ---- Simulator (software-in-the-loop harness)
add_library(grsimlib STATIC
  sim/src/world.cpp
  sim/src/sensors.cpp
  sim/src/scenario.cpp
  sim/src/runners.cpp
  sim/src/report.cpp
  sim/src/logs.cpp
)
target_include_directories(grsimlib PUBLIC sim/include)
target_link_libraries(grsimlib PUBLIC grcore)
target_compile_options(grsimlib PRIVATE -Wall -Wextra -Wno-unused-parameter)

find_package(Threads REQUIRED)
add_executable(grsim sim/apps/grsim.cpp)
target_link_libraries(grsim PRIVATE grsimlib Threads::Threads)

add_executable(grreplay sim/apps/grreplay.cpp)
target_link_libraries(grreplay PRIVATE grsimlib)

add_executable(grtrace sim/apps/grtrace.cpp)
target_link_libraries(grtrace PRIVATE grsimlib)

add_executable(grhil sim/apps/grhil.cpp)
target_link_libraries(grhil PRIVATE grsimlib Threads::Threads util)

# ---- Unit tests
enable_testing()
add_executable(grtests
  tests/test_main.cpp
  tests/test_core.cpp
  tests/test_sim.cpp
)
target_include_directories(grtests PRIVATE tests)
target_link_libraries(grtests PRIVATE grsimlib)
add_test(NAME unit COMMAND grtests)
