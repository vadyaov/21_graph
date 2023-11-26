#include "s21_graph_algorithms.h"

#include <iostream>
#include <limits>
#include <functional>
#include <set>

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

// цвета можно заменить на посещение\непосещение вершины, если не пригодятся
    // color[u] - цвет каждой вершины u из V[graph]
    // p[u] - предшественник вершины u
    // d[u] - расстояние от s до u
template<typename ContainerType>
  std::vector<int> BreadthDepthInterface(const Graph& graph, int s) {
    if (graph.Empty()) throw std::logic_error("Empty graph");
    if (s < 1 || static_cast<std::size_t>(s) > graph.Size())
      throw std::invalid_argument("Incorrect vertex number");

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

      // чтобы при поиске в глубину идти от меньшего индекса вершины к большему,
      // необходимо делать цикл в обратную сторону, но на цели алгоритма это не
      // влияет, поэтому не буду так делать пока что..
      // При выполнении поиска в глубину исследуются все ребра, выходящие из
      // вершины, открытой последней
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
    // момент который мне не нравится: из stack (поиск в глубину ведется от
    // старшей вершины к младшей, а хотелось бы идти по порядку
std::vector<int> GraphAlgorithms::DepthFirstSearch(const Graph &graph, int s) {
  return BreadthDepthInterface<s21::stack<int>>(graph, s);
}

    // usign queue
std::vector<int> GraphAlgorithms::BreadthFirstSearch(const Graph &graph, int s) {
  return BreadthDepthInterface<s21::queue<int>>(graph, s);
}

std::size_t GraphAlgorithms::GetShortestPathBetweenVertices(const Graph &graph,
                                                              int s, int f) {
  // d[v] - верхняя граница веса, которым обладает кратчайший путь из истока
  // v1 в вершину v2. d[v] - оценка кратчайшего пути
  std::vector<int> d(graph.Size(), std::numeric_limits<int>::max());
  /* std::vector<int> p(graph.Size(), -1); */
  d[s - 1] = 0;

  /* std::vector<int> S; */
  std::set<std::pair<int, int>> Q; // distance, vertex
  // в начале будут наименьшие ключи, yes ?
  Q.emplace(0, s - 1);


  while (!Q.empty()) {
    int u = Q.begin()->second;
    /* S.push_back(u); */
    Q.erase(Q.begin());

    for (std::size_t j = 0; j < graph.Size(); ++j) {
      int w = graph[u - 1][j];
      if (w != 0) {
        // relax edge
        if (d[j - 1] > d[u - 1] + w) {
          d[j - 1] = d[u - 1] + w;
          /* p[j - 1] = vertex - 1; */
          Q.emplace(d[j - 1], j - 1);
        }
      }

    }
  }

  for (auto i : d)
    std::cout << i << ' ';
  std::cout << std::endl;

  return d[f - 1];
}
