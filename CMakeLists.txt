cmake_minimum_required(VERSION 3.5)

project(serial)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(${CMAKE_PROJECT_NAME}_build_examples "Build examples" ON)

include_directories(include)

# library
add_library(${CMAKE_PROJECT_NAME} src/serial.c)

if (${CMAKE_PROJECT_NAME}_build_examples)
    add_subdirectory(examples)
endif ()

# install
install(TARGETS ${CMAKE_PROJECT_NAME}
        ARCHIVE DESTINATION lib)
file(GLOB BASE_HEADERS include/${CMAKE_PROJECT_NAME}/*.h)
install(FILES ${BASE_HEADERS} DESTINATION include/${CMAKE_PROJECT_NAME})
