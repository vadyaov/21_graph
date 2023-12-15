#include "s21_graph_algorithms.h"

#include <limits>
#include <queue>
#include <cmath>
#include <random>

#include "cpp2lib/stack.h"
#include "cpp2lib/queue.h"

enum class Color {WHITE, GRAY, BLACK};

struct Accessor {
  static int Get(const s21::stack<int>& s) {
    return s.top();
  }

  static int Get(const s21::queue<int>& s) {
    return s.front();
  }
};

template<typename ContainerType>
  std::vector<int> BreadthDepthInterface(const Graph& graph, int s) {
    if (graph.Empty()) throw std::invalid_argument("Empty graph");
    if (s < 1 || static_cast<std::size_t>(s) > graph.Size())
      throw std::out_of_range("Vertex range error");

    std::vector<Color> colors(graph.Size(), Color::WHITE);
    /* std::vector<int> p(graph.Size(), -1); */

    std::vector<int> path;

    colors[s - 1] = Color::GRAY;
    /* p[s - 1] = -1; */

    ContainerType container;
    container.push(s);

    while (!container.empty()) {
      int vertex = Accessor::Get(container);

      path.push_back(vertex);
      container.pop();

      /*
       * При выполнении поиска в глубину исследуются все ребра,
       * выходящие из вершины, открытой последней
       */
      for (std::size_t j = 0; j < graph.Size(); ++j) {
        if (graph[vertex - 1][j] && colors[j] == Color::WHITE) {
          colors[j] = Color::GRAY;
          container.push(j + 1);
        }
      }

      colors[vertex - 1] = Color::BLACK;
    }

    return path;
  }

std::vector<int> GraphAlgorithms::DepthFirstSearch(const Graph &graph, int s) {
  return BreadthDepthInterface<s21::stack<int>>(graph, s);
}

std::vector<int> GraphAlgorithms::BreadthFirstSearch(const Graph &graph, int s) {
  return BreadthDepthInterface<s21::queue<int>>(graph, s);
}

int GraphAlgorithms::GetShortestPathBetweenVertices(const Graph &graph, int s, int f) {
  const int sz = (int) graph.Size();
  if (sz == 0)
    throw std::invalid_argument("Empty graph");
  if (s < 1 || s > sz || f < 1 || f > sz)
    throw std::out_of_range("Vertex range error");

  std::vector<int> d(sz, std::numeric_limits<int>::max());
  d[s - 1] = 0;

  std::priority_queue<std::pair<int, int>> q;
  std::vector<Color> colors(sz, Color::WHITE);

  q.push(std::make_pair(0, s - 1));
  colors[s - 1] = Color::GRAY;

  while (!q.empty()) {
    int v = q.top().second;
    colors[v] = Color::BLACK;
    q.pop();

    for (int j = 0; j < sz; ++j) {
      int w = graph[v][j];
      if (w != 0 && d[j] > d[v] + w) {
        d[j] = d[v] + w;
        if (colors[j] == Color::WHITE) {
          q.push(std::make_pair(-d[j], j));
          colors[j] = Color::GRAY;
        }
      }
    }
  }

  if (d[f - 1] == std::numeric_limits<int>::max()) {
    throw std::runtime_error("Path between " + std::to_string(s) + " and " +
        std::to_string(f) + " does not exist.");
  }

  return d[f - 1];
}

std::vector<std::vector<int>>
GraphAlgorithms::GetShortestPathsBetweenAllVertices(const Graph& graph) {
  const int sz = graph.Size();
  if (sz == 0)
    throw std::invalid_argument("Empty graph");

  const int max_int = std::numeric_limits<int>::max();

  std::vector<std::vector<int>> d(sz, std::vector<int>(sz, max_int));

  for (int i = 0; i < sz; ++i) {
    for (int j = 0; j < sz; ++j) {
      if (graph[i][j])
        d[i][j] = graph[i][j];
    }
    d[i][i] = 0;
  }

  for (int k = 0; k < sz; ++k)
    for (int i = 0; i < sz; ++i)
      for (int j = 0; j < sz; ++j)
        if (d[i][k] < max_int && d[k][j] < max_int)
          d[i][j] = std::min(d[i][j], d[i][k] + d[k][j]);

  return d;
}

std::vector<std::vector<int>> GraphAlgorithms::GetLeastSpanningTree(const Graph& graph) {
  if (graph.Empty())
    throw std::invalid_argument("Empty graph");

  if (graph.IsDirect())
    throw std::invalid_argument("Graph must be undirected");

  const int sz = (int) graph.Size();
  const int max_int = std::numeric_limits<int>::max();

  std::vector<std::vector<int>> A(sz, std::vector<int>(sz, 0));

  std::vector<int> key(sz, max_int);
  key[0] = 0;

  std::vector<bool> used(sz);
  std::vector<int> p(sz, -1);

  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
    std::greater<std::pair<int, int>>> Q;
  Q.push(std::make_pair(key[0], 0));

  while (!Q.empty()) {
    int u = Q.top().second;
    Q.pop();

    if (used[u] == true) continue;
    if (key[u] == max_int)
      throw std::runtime_error("Ostov tree does not exist");

    used[u] = true;

    if (p[u] != -1)
      A[p[u]][u] = A[u][p[u]] = graph[u][p[u]];

    for (int j = 0; j != sz; ++j)
      if (graph[u][j] != 0)
        if (used[j] == false && graph[u][j] < key[j]) {
          key[j] = graph[u][j];
          p[j] = u;
          Q.push(std::make_pair(key[j], j));
        }

  }

  return A;
}

double RandomValue() {
  static std::random_device rd;
  static std::mt19937 engine(rd());
  static std::uniform_real_distribution<double> normal_distrib(0.0, 1.0);

  return normal_distrib(engine);
}

std::vector<std::vector<double>> NormalizedGraph(const Graph& graph) {
  const std::size_t sz = graph.Size();
  
  std::vector<std::vector<double>> normalized(sz, std::vector<double>(sz));

  for(std::size_t i = 0; i != sz; ++i)
    for(std::size_t j = 0; j != sz; ++j) {
      if ((graph[i][j] == 0 || graph[j][i] == 0) && i != j)
        throw std::runtime_error("Graph is not full");

      // line with magic value
      normalized[i][j] = 200.0 / graph[i][j];
    }

  return normalized;
}

int Roulette(const std::vector<double>& chance) {
    int next_point = -1;
    double cumulative_probability = 0.0;

    for (std::size_t i = 0; i < chance.size() && next_point == -1; ++i) {
        if (chance[i] > 0) {
            cumulative_probability += chance[i];
            if (RandomValue() <= cumulative_probability)
                next_point = static_cast<int>(i);
        }
    }

  return next_point;
}

GraphAlgorithms::TsmResult
MinimalSolution(const std::vector<GraphAlgorithms::TsmResult>& ants_data) {
  GraphAlgorithms::TsmResult min = ants_data[0];

  for (std::size_t i = 1; i != ants_data.size(); ++i) {
    if (ants_data[i].distance < min.distance)
      min = ants_data[i];
  }

  return min;
}

void UpdateFeromones(std::vector<std::vector<double>>& feromones,
                     std::vector<GraphAlgorithms::TsmResult>& paths, int size,
                     double Q, double reduce) {
  for (int i = 0; i != size; ++i)
    for (int j = 0; j != size; ++j)
      feromones[i][j] *= reduce;

  for (int ant = 0; ant < size; ++ant) {
    double delta_fero = Q / paths[ant].distance;
    for (std::size_t i = 0; i < paths[ant].vertices.size() - 1; ++i) {
      feromones[paths[ant].vertices[i]][paths[ant].vertices[i + 1]] += delta_fero;
    }
  }
}

GraphAlgorithms::TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(const Graph &graph) {
  const int sz = graph.Size();
  if (sz == 0)
    throw std::invalid_argument("Empty graph");

  TsmResult min_path {{}, std::numeric_limits<double>::max()};

  const double Q = 320.0;
  const double fero_reduce = 0.6;

  const double alpha = 1.0;
  const double beta = 4.0;

  std::vector<std::vector<double>> dist = NormalizedGraph(graph);
  std::vector<std::vector<double>> fero(sz, std::vector<double>(sz, 0.2));

  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {

    std::vector<TsmResult> ants_path(sz, {std::vector<int>(sz + 1, 0), 0});

    for (int ant = 0; ant < sz; ++ant) {
      int curr_point = ant;
      std::vector<bool> visited(sz, false);

      ants_path[ant].vertices[0] = ants_path[ant].vertices[sz] = curr_point;

      for (int i = 0; i < sz - 1; ++i) {
        visited[curr_point] = true;

        std::vector<double> wish(sz);
        for (int j = 0; j != sz; ++j)
          if (!visited[j])
            wish[j] = std::pow(fero[curr_point][j], alpha) *
                      std::pow(dist[curr_point][j], beta);

        double wish_sum = std::accumulate(wish.begin(), wish.end(), 0.0);

        std::vector<double> chance(sz);
        for (int j = 0; j != sz; ++j)
          chance[j] = wish[j] / wish_sum;

        int prev_point = curr_point;
        curr_point = Roulette(chance);

        if (curr_point == -1)
          throw std::runtime_error("Cannot find the solution");

        ants_path[ant].vertices[i + 1] = curr_point;
        ants_path[ant].distance += graph[prev_point][curr_point];
      }

      ants_path[ant].distance += graph[curr_point][ant];
    }

    UpdateFeromones(fero, ants_path, sz, Q, fero_reduce);

    if (min_path.distance > MinimalSolution(ants_path).distance)
      min_path = MinimalSolution(ants_path);

  }

  return min_path;
}
