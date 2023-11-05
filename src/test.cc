#include <iostream>

#include "s21_graph.h"

int main() {
  Graph graph;
  graph.LoadGraphFromFile("./examples/graph_0.txt");
  std::cout << graph << std::endl;

  std::cout << graph[10][9] << std::endl;

  return 0;
}
