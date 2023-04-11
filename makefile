# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -Iinclude

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
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(OBJECTS) $(TARGET)