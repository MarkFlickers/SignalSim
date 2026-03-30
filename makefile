# Makefile for SignalSim / JsonObsGen on Windows x86_64 (MinGW-w64)

TARGET      := bin/JsonObsGen.exe
BUILD_DIR   := build
OBJ_DIR     := $(BUILD_DIR)/obj

# CXX         ?= x86_64-w64-mingw32-g++
# AR          ?= x86_64-w64-mingw32-ar

# If you build inside MSYS2 MinGW64 shell, this also works:
CXX ?= g++
AR  ?= ar

CXXSTD      := -std=c++17
WARN        := -Wall -Wextra -Wpedantic
OPT         := -O0
DBG         := -g3
DEFS        := -DWIN32 -D_WINDOWS
INCLUDES    := -Iinc -Isrc -IJsonObsGen

# Add project-specific libs here if linker errors appear
# Typical Windows libs if networking/threads are used:
LIBS        := -lws2_32
LDFLAGS     :=

CXXFLAGS    := $(CXXSTD) $(WARN) $(OPT) $(DBG) $(DEFS) $(INCLUDES) -MMD -MP

SRC_DIRS    := src JsonObsGen
SOURCES     := $(foreach d,$(SRC_DIRS),$(wildcard $(d)/*.cpp))

# If JsonObsGen.cpp is the only entry point, keep all sources.
# If src/ also contains another main(), exclude it here:
# SOURCES := $(filter-out src/main.cpp,$(SOURCES))

OBJECTS     := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
DEPS        := $(OBJECTS:.o=.d)

.PHONY: all clean dirs print

all: dirs $(TARGET)

dirs:
	@mkdir -p bin
	@mkdir -p $(OBJ_DIR)/src
	@mkdir -p $(OBJ_DIR)/JsonObsGen

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS) $(LIBS)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) bin

print:
	@echo SOURCES=$(SOURCES)
	@echo OBJECTS=$(OBJECTS)

-include $(DEPS)
