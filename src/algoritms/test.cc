#include <iostream>

#include "s21_graph_algorithms.h"

int main() {
  Graph graph;
  graph.LoadGraphFromFile("../graph/examples/graph_0.txt");
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


  /* auto ostov = GraphAlgorithms::GetLeastSpanningTree(graph); */
  /* std::cout << "Ostov matrix: " << std::endl; */
  /* for (int i = 0, sz = ostov.size(); i != sz; ++i) { */
  /*   for (int j = 0; j != sz; ++j) */
  /*     std::cout << ostov[i][j] << " "; */
  /*   std::cout << std::endl; */
  /* } */

  auto res = GraphAlgorithms::SolveTravelingSalesmanProblem(graph);
  std::cout << "EXPECTED:\n";
  std::cout << " sequence of traversing vertices: 1-8-5-4-10-6-3-7-2-11-9-1\n"
    "route length: 253\n";

  std::cout << "GOT:\n";
  std::cout << " sequence of traversing vertices: ";
  for (auto i : res.vertices)
    std::cout << i << " ";

  std::cout << "\nroute length: " << res.distance << std::endl;
  

  return 0;
}
