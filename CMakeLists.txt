cmake_minimum_required(VERSION 3.12)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

project( MLPR )

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "MLPR" )

find_package( PCL REQUIRED)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "PCL not found")
endif()

include_directories( ${PCL_INCLUDE_DIRS} include )

file(GLOB_RECURSE SOURCES "src/*.cpp")

link_directories( ${PCL_LIBRARY_DIRS})

add_definitions( ${PCL_DEFINITIONS} )

# Set output directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(MLPR main.cpp ${SOURCES})

target_link_libraries( MLPR ${PCL_LIBRARIES} )

configure_file(test-final.csv ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-final.csv COPYONLY)
configure_file(train-final.csv ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/train-final.csv COPYONLY)