#include "s21_graph_algorithms.h"

#include <limits>
#include <queue>

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

int GraphAlgorithms::GetShortestPathBetweenVertices(const Graph &graph,
                                                              int s, int f) {
  const int sz = (int) graph.Size();
  if (s < 1 || s > sz - 1 || f < 1 || f > sz - 1)
    throw std::invalid_argument("Vertex is out of boundary");

  std::vector<int> d(sz, std::numeric_limits<int>::max());
  d[s - 1] = 0;

  std::priority_queue<std::pair<int, int>> q;
  q.push(std::make_pair(0, s - 1));

  while (!q.empty()) {
    int v = q.top().second, cur_d = -q.top().first;
    q.pop();
    if (cur_d > d[v]) continue;

    for (int j = 0; j < sz; ++j) {
      int w = graph[v][j];
      if (w != 0)
        if (d[j] > d[v] + w) {
          d[j] = d[v] + w;
          q.push(std::make_pair(-d[j], j));
        }
    }
  }

  return d[f - 1];
}

std::vector<std::vector<int>>
GraphAlgorithms::GetShortestPathsBetweenAllVertices(const Graph& graph) {
  const int sz = graph.Size();
  const int max_int = std::numeric_limits<int>::max();

  std::vector<std::vector<int>> d(sz, std::vector<int>(sz, max_int));
  for (int i = 0; i < sz; ++i)
    for (int j = 0; j < sz; ++j)
      if (graph[i][j]) d[i][j] = graph[i][j];

  for (int i = 0; i < sz; ++i)
    d[i][i] = 0;

  for (int k = 0; k < sz; ++k)
    for (int i = 0; i < sz; ++i)
      for (int j = 0; j < sz; ++j)
        if (d[i][k] < max_int && d[k][j] < max_int)
          d[i][j] = std::min(d[i][j], d[i][k] + d[k][j]);

  return d;
}

// вроде работает, только теперь надо возвращать матрицу смежности для остова
std::vector<int> GraphAlgorithms::GetLeastSpanningTree(const Graph& graph) {
  const int sz = (int) graph.Size();
  const int max_int = std::numeric_limits<int>::max();
  std::vector<int> A;

  std::vector<int> key(sz, max_int);
  key[0] = 0;  // starting from vertex No.0

  /* std::vector<int> p(sz, -1); // parents */

  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
    std::greater<std::pair<int, int>>> Q;
  Q.push(std::make_pair(key[0], 0));

  while (!Q.empty()) {
    int u = Q.top().second;
    std::cout << "size = " << Q.size() << "; " << Q.top().first << " " << Q.top().second << std::endl;
    Q.pop();

    if (std::find(A.begin(), A.end(), u) != A.end()) continue;
    A.push_back(u);

    std::cout << "A: ";
    for(int v : A)
      std::cout << v << " ";
    std::cout << std::endl;

    for (int j = 0; j != sz; ++j) {
      if (graph[u][j] != 0) {
        if (std::find(A.begin(), A.end(), j) == A.end() && graph[u][j] < key[j]) {
          key[j] = graph[u][j];
          std::cout << "pushing " << j << " with " << key[j] << std::endl;
          Q.push(std::make_pair(key[j], j));
        }
      }
    }
  }

  /* std::cout << "First element iq Q: " << Q.top().first << std::endl; */

  return A;
}
