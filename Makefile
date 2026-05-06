# ===============================
# Makefile for Windows + MinGW
# ===============================
CC  = gcc
CXX = g++
CFLAGS   = -Wall -Wextra -std=c11
CXXFLAGS = -Wall -Wextra -std=c++17

# ===============================
# Libs
# ===============================
LIBS = -lssl -lcrypto

# ===============================
# Detect OS
# ===============================
ifeq ($(OS),Windows_NT)
    EXE_EXT	= .exe
else
    EXE_EXT =
endif

# ===============================
# Source directories
# EDIT ONLY HERE!!!!!!!!!!!
# ===============================
SRC_DIRS  = src utils \
			src/app/cli \
			src/plant \
			src/plant/models \
			src/controller \
			src/controller/pid \
			src/controller/fuzzy \
			src/simulation \
			src/logger \
			src/visualization

TEST_DIRS = test
ASSETS_DIR = assets
MAIN = src/main.cpp

# ===============================
# Other directories
# ===============================
BUILD_DIR     = build
OBJ_DIR       = $(BUILD_DIR)/obj
COVERAGE_DIR  = $(BUILD_DIR)/coverage
CPPCHECK_DIR  = $(BUILD_DIR)/cppcheck
BUILD_ASSETS_DIR = $(BUILD_DIR)/assets

# ===============================
# Windows \ → /
# ===============================
ifeq ($(OS),Windows_NT)
	SRC_DIRS  := $(subst /,\,$(SRC_DIRS))
	TEST_DIRS := $(subst /,\,$(TEST_DIRS))
	BUILD_DIR := $(subst /,\,$(BUILD_DIR))
	OBJ_DIR   := $(subst /,\,$(OBJ_DIR))
	COVERAGE_DIR := $(subst /,\,$(COVERAGE_DIR))
	CPPCHECK_DIR := $(subst /,\,$(CPPCHECK_DIR))
endif

# ===============================
# Search for all the .c inside 
# the src directories
# ===============================
C_SRC_FILES   	= $(foreach d,$(SRC_DIRS),$(wildcard $(d)/*.c))
CPP_SRC_FILES	= $(foreach d,$(SRC_DIRS),$(wildcard $(d)/*.cpp))
C_TEST_FILES 	= $(foreach d,$(TEST_DIRS),$(wildcard $(d)/test_*.c))
CPP_TEST_FILES 	= $(foreach d,$(TEST_DIRS),$(wildcard $(d)/test_*.cpp))

# ===============================
# Automatic Includes
# ===============================
CFLAGS += $(foreach dir,$(SRC_DIRS), -I$(dir))
CXXFLAGS += $(foreach d,$(SRC_DIRS), -I$(d))

# ===============================
# Outputs
# ===============================
TARGET       = $(BUILD_DIR)/main$(EXE_EXT)
TEST_TARGET  = $(BUILD_DIR)/tests$(EXE_EXT)

# ===============================
# Object files
# ===============================
OBJ_C_SRC	= $(patsubst %.c,$(OBJ_DIR)/%.o,$(C_SRC_FILES))
OBJ_CPP_SRC = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(CPP_SRC_FILES))
OBJ_C_TEST 	= $(patsubst %.c,$(OBJ_DIR)/%.o,$(C_TEST_FILES))
OBJ_CPP_TEST= $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(CPP_TEST_FILES))

OBJ_ALL = $(OBJ_C_SRC) $(OBJ_CPP_SRC)
OBJ_ALL_TEST = $(OBJ_C_TEST) $(OBJ_CPP_TEST)

# ===============================
# Create directories automatically
# ===============================

# Ensure build dirs exist before compiling
.dirs:
ifeq ($(OS),Windows_NT)
	if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)
	if not exist $(OBJ_DIR) mkdir $(OBJ_DIR)
	for %%d in ($(SRC_DIRS)) do if not exist "$(OBJ_DIR)\%%d" mkdir "$(OBJ_DIR)\%%d"
	for %%d in ($(TEST_DIRS)) do if not exist "$(OBJ_DIR)\%%d" mkdir "$(OBJ_DIR)\%%d"
	xcopy $(ASSETS_DIR) $(BUILD_ASSETS_DIR) /E /I /Y
else
	mkdir -p $(BUILD_DIR)
	mkdir -p $(OBJ_DIR)
	@for d in $(SRC_DIRS); do \
		mkdir -p "$(OBJ_DIR)/$$d"; \
	done
	@for d in $(TEST_DIRS); do \
		mkdir -p "$(OBJ_DIR)/$$d"; \
	done
	mkdir -p $(BUILD_ASSETS_DIR)
	cp -r $(ASSETS_DIR)/* $(BUILD_ASSETS_DIR)/
endif

# =================================
# Build rules
# =================================
all: .dirs $(TARGET)

# Link with C++ to avoid unresolved C++ symbols
$(TARGET): $(OBJ_ALL)
	$(CXX) $(CXXFLAGS) $(OBJ_ALL) $(LIBS) -o $(TARGET)
	@echo ==== Compilation completed: $(TARGET) ====

# Compile C files
$(OBJ_DIR)/%.o: %.c | .dirs
	$(CC) $(CFLAGS) -c $< -o $@

# Compile C++ files
$(OBJ_DIR)/%.o: %.cpp | .dirs
	$(CXX) $(CXXFLAGS) -c $< -o $@

# =================================
# Build tests (no main.cpp)
# =================================
tests: .dirs $(OBJ_ALL) $(OBJ_ALL_TEST)
	$(CXX) $(CXXFLAGS) \
		$(filter-out $(OBJ_DIR)/$(MAIN:.cpp=.o),$(OBJ_ALL)) \
		$(OBJ_ALL_TEST) $(LIBS) -o $(TEST_TARGET)
	@echo ==== Tests built: $(TEST_TARGET) ====

# =================================
# Run tests
# =================================
run-tests: tests
	@echo ==== Executing tests ====
	@./$(TEST_TARGET)

# ===============================
# Valgrind (Linux only)
# ===============================
valgrind: 
ifeq ($(OS),Windows_NT)
	@echo Valgrind is not available on Windows
else
	@echo ==== Running under Valgrind ====
	$(MAKE) tests
	valgrind --leak-check=full ./$(TEST_TARGET)
endif

# ===============================
# Coverage (gcov + gcovr)
# ===============================
coverage:
	@echo ==== Generating coverage ====
ifeq ($(OS),Windows_NT)
	$(MAKE) clean
	if not exist $(COVERAGE_DIR) mkdir $(COVERAGE_DIR)
	$(MAKE) CXXFLAGS="$(CXXFLAGS) -fprofile-arcs -ftest-coverage" tests
	./$(TEST_TARGET)
else
	$(MAKE) clean
	mkdir -p $(COVERAGE_DIR)
	$(MAKE) CXXFLAGS="$(CXXFLAGS) -fprofile-arcs -ftest-coverage" tests
	./$(TEST_TARGET)
	gcovr --html-details --output $(COVERAGE_DIR)/coverage.html
endif

# ===============================
# Cppcheck (Static Analysis)
# ===============================
cppcheck:
	@echo ==== Running Cppcheck ====
ifeq ($(OS),Windows_NT)
	@echo Cppcheck build option has not yet tested on Windows
else
	mkdir -p $(CPPCHECK_DIR)
endif
	cppcheck $(CPPCHECK_OPTS) $(SRC_DIRS) \
		--enable=all --inconclusive --std=c11 --std=c++17 \
        --inline-suppr --force \
		--suppress=missingIncludeSystem \
		--check-level=exhaustive \
		--addon=tools/cppcheck/misra.json \
		--cppcheck-build-dir=$(CPPCHECK_DIR) \
		--xml --xml-version=2 \
		--checkers-report=$(CPPCHECK_DIR)/checkers.txt \
		--output-file=$(CPPCHECK_DIR)/cppcheck-result.xml
	@echo ==== Cppcheck finished. Report in $(CPPCHECK_DIR)/cppcheck-result.xml ====
	@echo ==== Generating HTML Report ====
	cppcheck-htmlreport \
		--file=$(CPPCHECK_DIR)/cppcheck-result.xml \
		--report-dir=$(CPPCHECK_DIR)/html
	@echo ==== HTML report ready in $(CPPCHECK_DIR)/html ====

# =================================
# Run main application
# =================================
run: $(TARGET)
	@echo ==== Running Application ====
	@./$(TARGET)

# =================================
# Clean everything
# =================================
clean:
	@echo ==== Cleaning build directory ====
ifeq ($(OS),Windows_NT)
	if exist $(BUILD_DIR) rmdir /S /Q $(BUILD_DIR)
else
	-rm -rf $(BUILD_DIR)
endif
