# Define the compiler and flags
CXX = g++
CXXFLAGS = 	$(shell pkg-config --cflags opencv4)
CXXFLAGS += --std=c++17 -g -Wall -Werror -pedantic 
LDFLAGS += $(shell pkg-config --libs opencv4)

# Default target to build the executable
all: a_star

# Rule to link the executable
a_star: a_star.cpp
	$(CXX) $(CXXFLAGS) a_star.cpp -o $@ $(LDFLAGS)

# Clean up build artifacts
clean:
	rm -f a_star.exe

# # Compiler
# CXX = g++

# # Compiler flags
# CXXFLAGS = -I/usr/include/opencv4 -I/usr/include -g

# # Linker flags
# LDFLAGS = -L/usr/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

# # Target executable
# TARGET = a_star

# # Source files
# SOURCES = a_star.cpp

# # Object files
# OBJECTS = $(SOURCES:.cpp=.o)

# all: $(TARGET)

# $(TARGET): $(OBJECTS)
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# %.o: %.cpp
# 	$(CXX) -c $< -o $@ $(CXXFLAGS)

# clean:
# 	rm -f $(TARGET) $(OBJECTS)