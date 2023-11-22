#include <iostream>

#include "s21_graph.h"

int main() {
  Graph graph;
  graph.LoadGraphFromFile("./examples/graph_2.txt");
  std::cout << graph << std::endl;

  if (graph.Directed())
    std::cout << "This graph is directed\n";
  else
    std::cout << "This graph is undirected\n";

  graph.ExportGraphToDot("graphname.dot");

  return 0;
}
