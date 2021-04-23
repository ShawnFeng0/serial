project(serial_examples)

cmake_minimum_required(VERSION 3.5)

# serial library
link_libraries(serial)
include_directories(PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Examples
add_executable(serial_examples_read_write read_write.cpp)
