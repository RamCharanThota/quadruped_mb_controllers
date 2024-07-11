# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(tsid_formulation_example)

# Set the Python executable
set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE PATH "Path to the Python 3 executable")

# Find Python3 package
find_package(Python3 REQUIRED COMPONENTS Development)

# Print the Python paths to verify they're set correctly
message(STATUS "Python3 executable: ${Python3_EXECUTABLE}")
message(STATUS "Python3 include dir: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 libraries: ${Python3_LIBRARIES}")

# Find the tsid package
find_package(tsid REQUIRED)

# Set the source directory
set(TSID_SOURCE_DIR "${CMAKE_SOURCE_DIR}")

# Print the TSID_SOURCE_DIR to verify it's set correctly
message(STATUS "TSID_SOURCE_DIR is set to ${TSID_SOURCE_DIR}")

# Specify the source files
set(SOURCES
    ${TSID_SOURCE_DIR}/tsid_formulation.cpp
)

# Add an executable with the above sources
add_executable(tsid_formulation_example ${SOURCES})

target_link_libraries(tsid_formulation_example tsid)

# Include the tsid headers
target_include_directories(tsid_formulation_example PRIVATE ${TSID_INCLUDE_DIRS})
