#include "s21_graph_algorithms.h"

#include <limits>
#include <queue>
#include <cmath>
#include <random>
#include <numeric>

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
    if (graph.Empty()) throw std::invalid_argument("Empty graph");
    if (s < 1 || static_cast<std::size_t>(s) > graph.Size())
      throw std::out_of_range("Vertex range error");

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

      /*
       * При выполнении поиска в глубину исследуются все ребра, выходящие из
       * вершины, открытой последней
       */
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
      if (w != 0)
        if (d[j] > d[v] + w) {
          d[j] = d[v] + w;
          if (colors[j] == Color::WHITE) {
            q.push(std::make_pair(-d[j], j));
            colors[j] = Color::GRAY;
          }
        }
    }
  }

  if (d[f - 1] == std::numeric_limits<int>::max())
    throw std::runtime_error("Path between " + std::to_string(s) + " and " +
        std::to_string(f) + " does not exist.");

  return d[f - 1];
}

std::vector<std::vector<int>>
GraphAlgorithms::GetShortestPathsBetweenAllVertices(const Graph& graph) {
  const int sz = graph.Size();
  if (sz == 0)
    throw std::invalid_argument("Empty graph");

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

std::vector<std::vector<int>> GraphAlgorithms::GetLeastSpanningTree(const Graph& graph) {
  const int sz = (int) graph.Size();
  if (sz == 0)
    throw std::invalid_argument("Empty graph");

  // or make undirect graph from direct, removing const modifier
  if (graph.IsDirect())
    throw std::invalid_argument("Graph must be undirected");

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
  /* const double min = graph.MinWeight(); */
  /* const double max = graph.MaxWeight(); */
  
  std::vector<std::vector<double>> normalized(sz, std::vector<double>(sz));

  for(std::size_t i = 0; i != sz; ++i)
    for(std::size_t j = 0; j != sz; ++j) {
      if (graph[i][j] == 0 && graph[j][i] == 0 && i != j)
        throw std::runtime_error("Graph is not full");

      normalized[i][j] = 200.0 / graph[i][j]; // (graph[i][j] - min) / (max - min);
    }

  return normalized;
}

int Roulette(const std::vector<double>& chance) {
    int next_point = -1;
    double cumulative_probability = 0.0;

    for (std::size_t i = 0; i < chance.size(); ++i) {
        if (chance[i] > 0) {
            cumulative_probability += chance[i];
            if (RandomValue() <= cumulative_probability) {
                next_point = static_cast<int>(i);
                break;
            }
        }
    }
  /* std::cout << "next point will be " << next_point << "\n\n"; */
  return next_point;
}

GraphAlgorithms::TsmResult
MinimalSolution (const std::vector<GraphAlgorithms::TsmResult>& ants_data) {
  GraphAlgorithms::TsmResult min = ants_data[0];

  for (std::size_t i = 1; i != ants_data.size(); ++i) {
    if (ants_data[i].distance < min.distance)
      min = ants_data[i];
  }

  return min;
}

GraphAlgorithms::TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(const Graph &graph) {
  const int sz = graph.Size();
  if (sz == 0)
    throw std::invalid_argument("Empty graph");

  TsmResult min_path {{}, std::numeric_limits<double>::max()};
  constexpr int MAX_ITERATIONS = 100;

  const int ants = sz;

  const double alpha = 1.0;
  const double beta = 4.0;
  const double Q = 320.0;

  const double fero_reduce = 0.6;

  std::vector<std::vector<double>> dist = NormalizedGraph(graph);
  std::vector<std::vector<double>> fero(sz, std::vector<double>(sz, 0.2));

  std::vector<TsmResult> ants_path;

  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {

    ants_path = std::vector<TsmResult>(ants, {std::vector<int>(sz + 1, 0), 0});

    for (int ant = 0; ant < ants; ++ant) {
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

    for (int i = 0; i != sz; ++i)
      for (int j = 0; j != sz; ++j)
        fero[i][j] *= fero_reduce;

    for (int ant = 0; ant < ants; ++ant) {
      double delta_fero = Q / ants_path[ant].distance;
      for (std::size_t i = 0; i < ants_path[ant].vertices.size() - 1; ++i) {
        fero[ants_path[ant].vertices[i]][ants_path[ant].vertices[i + 1]] += delta_fero;
      }
    }

    if (min_path.distance > MinimalSolution(ants_path).distance)
      min_path = MinimalSolution(ants_path);

  }

  return min_path;
}

        /* std::cout << "Distance between " << prev_point << " and " << curr_point << " is " << dist[prev_point][curr_point] << std::endl; */


  /* std::cout << "Distances: \n"; */
  /* for (int i = 0; i < sz; ++i) { */
  /*   fero[i][i] = 0; */
  /*   for (int j = 0; j < sz; ++j) */
  /*     std::cout << dist[i][j] << " "; */
  /*   std::cout << std::endl; */
  /* } */
  /* std::cout << std::endl; */

    /* for (std::size_t p = 0; p < ants_path.size(); ++p) { */
    /*   std::cout << "\nANT " << p << ":\npath: "; */
    /*   for (int v : ants_path[p].vertices) */
    /*     std::cout << v << " "; */
    /*   std::cout << "\ndistance: " << ants_path[p].distance << "\n"; */
    /* } */


        /* std::cout << "chances: "; */
        /* for (auto p : chance) */
        /*   std::cout << p << " "; */
        /* std::cout << std::endl; */
        /* std::cout << "chances: "; */
        /* for (auto p : chance) */
        /*   std::cout << p << " "; */
        /* std::cout << std::endl; */

        /* std::cout << "wishes: "; */
        /* for (auto w : wish) */
        /*   std::cout << w << " "; */
        /* std::cout << std::endl; */

/*   std::cout << "FEROMONES:\n"; */
/*   for (int i = 0; i != sz; ++i) { */
/*     for (int j = 0; j != sz; ++j) */
/*       std::cout << fero[i][j] << " "; */
/*     std::cout << std::endl; */
/*   } */

