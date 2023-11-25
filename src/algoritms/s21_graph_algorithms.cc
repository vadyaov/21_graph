#include "s21_graph_algorithms.h"

#include <iostream>
#include "stack.h"
#include "queue.h"

    // usign stack
std::vector<int> GraphAlgorithms::DepthFirstSearch(const Graph &graph, int s) {
  if (graph.Empty()) throw std::logic_error("Empty graph");

  std::cout << "Now i'm at verttex No. " << s << std::endl;
  /* const std::size_t size = graph.Size(); */
  /* Color colors[size]; */

  /* for (std::size_t i = 0; i < size; ++i) */
  /*   colors[i] = Color::WHITE; */

  std::vector<int> path;


  return path;
}

    // usign queue
    // color[u] - цвет каждой вершины u из V[graph]
    // p[u] - предшественник вершины u
    // d[u] - расстояние от s до u
std::vector<int> GraphAlgorithms::BreadthFirstSearch(const Graph &graph, int s) {

  std::vector<Color> color(graph.Size(), Color::WHITE);
  /* std::vector<int> d(graph.Size(), -1); */
  /* std::vector<int> p(graph.Size(), -1); */

  std::vector<int> path;

  color[s - 1] = Color::GRAY;
  /* d[s - 1] = 0; */
  /* p[s - 1] = -1; */

  s21::queue<int> Q;

  Q.push(s);

  while (!Q.empty()) {
    int vertex = Q.front();
    std::cout << "Vertex = " << vertex << std::endl;
    path.push_back(vertex);
    Q.pop();
    for (std::size_t j = 0; j < graph.Size(); ++j) {
      if (graph[vertex - 1][j] && color[j] == Color::WHITE) {
        color[j] = Color::GRAY;
        /* d[j] = d[vertex - 1] + 1; */
        /* p[j] = vertex - 1; */
        Q.push(j + 1);
      }
    }
    color[vertex - 1] = Color::BLACK;
  }

  return path;
}
