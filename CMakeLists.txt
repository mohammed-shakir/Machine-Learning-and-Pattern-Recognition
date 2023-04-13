cmake_minimum_required(VERSION 3.12)  # Make sure you have the right minimum required version

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# Create Project
project( MLPR )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "MLPR" )

# Find Packages
find_package( PCL REQUIRED)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "PCL not found")
endif()

# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} include )

file(GLOB_RECURSE SOURCES "src/*.cpp")

# For Use Not PreCompiled Features 
#add_definitions( -DPCL_NO_PRECOMPILE )

# [Linker]>[General]>[Additional Library Directories]
link_directories( ${PCL_LIBRARY_DIRS})

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

add_executable(MLPR main.cpp ${SOURCES})

# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries( MLPR ${PCL_LIBRARIES} )

configure_file(test-final.csv ${CMAKE_CURRENT_BINARY_DIR}/Debug/test-final.csv COPYONLY)
configure_file(train-final.csv ${CMAKE_CURRENT_BINARY_DIR}/Debug/train-final.csv COPYONLY)
