# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -Iinclude -I"$(PCL_ROOT)\3rdParty\Boost\include\boost-1_80" -I"${PCL_ROOT}\3rdParty\FLANN\include\flann -I

# PCL
PCL_INCLUDE = "$(PCL_ROOT)\include\pcl-1.13"  # Change the version number to match your PCL version
PCL_LIB = "$(PCL_ROOT)\lib"
PCL_DEFINITIONS = -DPCL_NO_PRECOMPILE
PCL_LIBRARIES = -lpcl_common -lpcl_io -lpcl_visualization -lpcl_filters -lpcl_features -lpcl_search -lpcl_sample_consensus -lpcl_kdtree -lpcl_octree -lpcl_segmentation -lpcl_surface -lpcl_registration -lpcl_keypoints -lpcl_tracking -lpcl_recognition -lpcl_ml -lpcl_outofcore -lpcl_people

# Folders
INCLUDE_DIR = include
SRC_DIR = src

# Output executable
TARGET = MLPR

# Source and object files
SOURCES = $(wildcard $(SRC_DIR)/*.cpp) main.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Rules
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ -I$(PCL_INCLUDE) -L$(PCL_LIB) $(PCL_LIBRARIES) $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ -I$(PCL_INCLUDE) -L$(PCL_LIB) $(PCL_LIBRARIES)

.PHONY: clean
clean:
	rm -f $(OBJECTS) $(TARGET)
