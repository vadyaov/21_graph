CXX = g++
CXXFLAGS = -Wall -Werror -Wextra -Wpedantic -std=c++17 #-fsanitize=address -g

GRAPH_DIR = ./graph
GRAPH_SRC = $(GRAPH_DIR)/s21_graph.cc
GRAPH_OBJ = $(GRAPH_DIR)/s21_graph.o

ALGO_DIR = ./algorithms
ALGO_SRC = $(ALGO_DIR)/s21_graph_algorithms.cc
ALGO_OBJ = $(ALGO_DIR)/s21_graph_algorithms.o

GCOV_TARGET = ./coverage
COVERFLAGS = -fprofile-arcs -ftest-coverage

all: lib test

lib: s21_graph.a s21_graph_algorithms.a

test: lib
	@mkdir -p examples/dots/
	@cmake -DCMAKE_CXX_FLAGS="$(CXXFLAGS)" -S ./tests/ -B ./tests/build
	@cmake --build ./tests/build
	@./tests/build/test_graph

gcov_report: $(GCOV_TARGET)

$(GCOV_TARGET): CXXFLAGS += $(COVERFLAGS)
$(GCOV_TARGET): test Makefile
	lcov --rc lcov_branch_coverage=1 --capture --directory . --output-file coverage.info \
		--no-external --exclude "$(CURDIR)/tests/*"
	genhtml coverage.info --output-directory coverage_report
	open ./coverage_report/index.html

s21_graph.a: $(GRAPH_OBJ)
	ar -rcs $@ $< 

s21_graph_algorithms.a: $(ALGO_OBJ)
	ar -rcs $@ $<

$(GRAPH_OBJ): $(GRAPH_SRC)
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(ALGO_OBJ): $(ALGO_SRC)
	$(CXX) -c $(CXXFLAGS) $< -o $@

app: s21_graph.a s21_graph_algorithms.a
	$(CXX) $(CXXFLAGS) console/*.cc $? -lncurses -o $@.out
	./$@.out

clean:
	@rm -f $(GRAPH_OBJ) $(ALGO_OBJ) *.a
	@rm -rf tests/build/ coverage*
	@rm -f graph/*gcno algorithms/*gcda
	@rm -f graph/*gcda algorithms/*gcno
	@rm -f ./examples/dots/*
	@rm -f app.out
