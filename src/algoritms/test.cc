#include <iostream>

#include "s21_graph_algorithms.h"

int main() {
  Graph graph;
  graph.LoadGraphFromFile("../graph/examples/prim.txt");
  std::cout << graph << std::endl;

  if (graph.IsDirect())
    std::cout << "This graph is directed\n";
  else
    std::cout << "This graph is undirected\n";

  graph.ExportGraphToDot("graphname.dot");

  /* std::cout << "Breadth:\n"; */
  /* for (auto i : GraphAlgorithms::BreadthFirstSearch(graph, 1)) */
  /*   std::cout << i << ' '; */

  /* std::cout << "\nDepth:\n"; */
  /* for (auto i : GraphAlgorithms::DepthFirstSearch(graph, 1)) */
  /*   std::cout << i << ' '; */

  /* std::cout << std::endl; */

  /* std::cout << "Dejkstra:\n"; */

  /* std::cout << "Distance = " << GraphAlgorithms::GetShortestPathBetweenVertices(graph, 1, 4) << std::endl; */

  /* auto dsts = GraphAlgorithms::GetShortestPathsBetweenAllVertices(graph); */
  /* for (int i = 0, sz = dsts.size(); i < sz; ++i) { */
  /*   for (int j = 0; j < sz; ++j) */
  /*     std::cout << dsts[i][j] << ' '; */
  /*   std::cout << std::endl; */
  /* } */


  for (int i : GraphAlgorithms::GetLeastSpanningTree(graph))
    std::cout << i << ' ';
  std::cout << std::endl;

  return 0;
}
