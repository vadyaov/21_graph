CXX = g++
CXXFLAGS = -Wall -Werror -Wextra -Wpedantic -std=c++17

GRAPH_DIR = ./graph
GRAPH_SRC = $(GRAPH_DIR)/s21_graph.cc
GRAPH_OBJ = $(GRAPH_DIR)/s21_graph.o

# GTEST_DIR = 
# GTEST_LIB = /usr/lib/x86_64-linux-gnu/libgtest.a
# GTEST_MAIN_LIB = /usr/lib/x86_64-linux-gnu/libgtest_main.a

all: s21_graph.a #test

test: s21_graph.a
	cmake -S ./tests/ -B ./tests/build
	cmake --build ./tests/build
	cd ./tests/build && ctest

s21_graph.a: $(GRAPH_OBJ)
	ar -rcs $@ $< 

$(GRAPH_OBJ): $(GRAPH_SRC)
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean:
	rm -f $(GRAPH_OBJ)
