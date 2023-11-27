#include "s21_graph_algorithms.h"

/* #include <functional> */
#include <limits>

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

int ExtractMin(std::vector<int>& Q, const std::vector<int>& d) {
  int min {d[0]};
  int min_index {0};

  for (std::size_t i = 0; i < Q.size(); ++i) {
    if (d[Q[i]] < min) {
      min = d[Q[i]];
      min_index = i;
    }
  }
  std::cout << "MIN_INDEX = " << min_index << std::endl;

  int vertex = Q[min_index];
  Q.erase(Q.begin() + min_index);
  return vertex;
}

// need to improve a lot
// 1. check s and f
// 2. check for negative weights
// 3. Q and d are bad implementation of priority queue
std::size_t GraphAlgorithms::GetShortestPathBetweenVertices(const Graph &graph,
                                                              int s, int f) {
  std::vector<int> Q; // vertex
  std::vector<int> d(graph.Size(), std::numeric_limits<int>::max());
  for (std::size_t i = 0; i < graph.Size(); ++i)
    Q.push_back(i);

  d[s - 1] = 0;
  /* std::vector<int> S; */

  while (!Q.empty()) {
    std::cout << "Q:\n";
    for (int i : Q)
      std::cout << i << ' ';
    std::cout << std::endl;

    std::cout << "d:\n";
    for (int i : d)
      std::cout << i << ' ';
    std::cout << std::endl;


    int u = ExtractMin(Q, d);
    std::cout << "u = " << u << std::endl;
    /* S.push_back(u); */
    if (d[u] == std::numeric_limits<int>::max()) continue;

    for (std::size_t j = 0; j < graph.Size(); ++j) {
      int w = graph[u][j];
      if (w != 0) {
        // relax edge
        if (d[j] > d[u] + w) {
          d[j] = d[u] + w;
          /* p[j - 1] = vertex - 1; */
        }
      }

    }
  }

  std::cout << std::endl;
  std::cout << std::endl;
  for (auto i : d)
    std::cout << i << ' ';
  std::cout << std::endl;

  return d[f - 1];
}
