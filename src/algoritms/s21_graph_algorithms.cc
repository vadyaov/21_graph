#include "s21_graph_algorithms.h"

#include <iostream>
#include "stack.h"
#include "queue.h"

enum class Color {WHITE, GRAY, BLACK};

struct Accessor {
  static int Get(const s21::stack<int>& s) {
    return s.top();
  }

  static int Get(const s21::queue<int>& s) {
    return s.front();
  }
};

// цвета можно заменить на посещение\непосещение вершины
template<typename ContainerType>
  std::vector<int> BreadthDepthInterface(const Graph& graph, int s) {
    std::vector<Color> color(graph.Size(), Color::WHITE);
    /* std::vector<int> p(graph.Size(), -1); */

    std::vector<int> path;

    color[s - 1] = Color::GRAY;
    /* p[s - 1] = -1; */

    ContainerType container;
    container.push(s);

    while (!container.empty()) {
      int vertex = Accessor::Get(container);

      path.push_back(vertex);
      container.pop();

      for (std::size_t j = 0; j < graph.Size(); ++j) {
        if (graph[vertex - 1][j] && color[j] == Color::WHITE) {
          color[j] = Color::GRAY;
          container.push(j + 1);
        }
      }

      color[vertex - 1] = Color::BLACK;
    }

    return path;
  }

    // usign stack
std::vector<int> GraphAlgorithms::DepthFirstSearch(const Graph &graph, int s) {
  /* if (graph.Empty()) throw std::logic_error("Empty graph"); */

  return BreadthDepthInterface<s21::stack<int>>(graph, s);
}

    // usign queue
    // color[u] - цвет каждой вершины u из V[graph]
    // p[u] - предшественник вершины u
    // d[u] - расстояние от s до u
std::vector<int> GraphAlgorithms::BreadthFirstSearch(const Graph &graph, int s) {

  return BreadthDepthInterface<s21::queue<int>>(graph, s);
}
