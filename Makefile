# =========================================================
# Makefile for WeaklyHard (multi-executable)
# =========================================================

CONFIG ?= Release
CXX = g++

CPLEX_DIR ?= /opt/ibm/ILOG/CPLEX_Studio221

# ---- Includes ----
INCLUDES = \
	-I$(CPLEX_DIR)/cplex/include \
	-I$(CPLEX_DIR)/concert/include

# ---- Libraries ----
LIBDIRS = \
	-L$(CPLEX_DIR)/cplex/lib/x86-64_linux/static_pic \
	-L$(CPLEX_DIR)/concert/lib/x86-64_linux/static_pic

LIBS = \
	-lilocplex \
	-lconcert \
	-lcplex \
	-lm \
	-lpthread \
	-ldl

# ---- Sources (shared code only) ----
SRCS = \
	c_utility.cpp \
	MILP.cpp \
	mixed_tasks.cpp \
	weaklydatasetJSON.cpp

OBJS = $(SRCS:.cpp=.o)

# ---- Main targets ----
EXP_TARGET   = WeaklyHard
CASE_TARGET  = WeaklyHardCase

# ---- Flags ----
CXXFLAGS_COMMON = \
	-std=c++17 \
	-Wall \
	-Wextra \
	-fPIC \
	$(INCLUDES)

ifeq ($(CONFIG),Debug)
CXXFLAGS = $(CXXFLAGS_COMMON) -g -O0 -DDEBUG
else
CXXFLAGS = $(CXXFLAGS_COMMON) -O3 -DNDEBUG
endif

# =========================================================
# Build all
# =========================================================
all: $(EXP_TARGET) $(CASE_TARGET)

# =========================================================
# Experiment binary
# =========================================================
$(EXP_TARGET): $(OBJS) main_experiment.o
	$(CXX) $^ -o $@ $(LIBDIRS) $(LIBS)

# =========================================================
# Case study binary
# =========================================================
$(CASE_TARGET): $(OBJS) main_case.o
	$(CXX) $^ -o $@ $(LIBDIRS) $(LIBS)

# =========================================================
# Compile rule
# =========================================================
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# =========================================================
# Clean
# =========================================================
clean:
	rm -f $(OBJS) *.o $(EXP_TARGET) $(CASE_TARGET)

run-exp: $(EXP_TARGET)
	./$(EXP_TARGET)

run-case: $(CASE_TARGET)
	./$(CASE_TARGET)

# =========================================================
# Usage:
#   make
#   make run-exp
#   make run-case
# =========================================================
