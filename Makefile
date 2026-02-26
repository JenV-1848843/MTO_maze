# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Iinclude
LDFLAGS = -lgpiod

# Directories
SRC_DIR = src
BUILD_DIR = build
INCLUDE_DIR = include

# Source files
SOURCES = $(SRC_DIR)/main.cpp # $(SRC_DIR)/servo_control.cpp
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Target executable
TARGET = $(BUILD_DIR)/run_me

# Default target
all: $(BUILD_DIR) $(TARGET)

# Create build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Link object files to create executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile source files to object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(INCLUDE_DIR)/%.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Special rule for main.cpp (no matching .h file)
$(BUILD_DIR)/main.o: $(SRC_DIR)/main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)/*.o $(TARGET)
	@echo "Cleaned build directory"

# Run the program
run: $(TARGET)
	sudo $(TARGET)

# Phony targets
.PHONY: all clean run