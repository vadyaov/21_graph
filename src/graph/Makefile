CXX = g++
CXXFLAGS = -Wall -Werror -Wextra -Wpedantic -std=c++17

all: s21_graph.a #test

clean:
	rm -f *.o *.a

test: test.cc s21_graph.cc
	$(CXX) $(CXXFLAGS) s21_graph.cc test.cc -o test
	./test

s21_graph.a: s21_graph.o
	ar -rcs $@ $< 

s21_graph.o: s21_graph.cc
	$(CXX) -c $(CXXFLAGS) $< -o $@

