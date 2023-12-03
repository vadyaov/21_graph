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

std::vector<std::vector<int>> GraphAlgorithms::GetLeastSpanningTree(const Graph& graph) {
  // or make undirect graph from direct, removing const modifier
  if (graph.IsDirect())
    throw std::invalid_argument("Graph must be undirected.");

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
  std::random_device rd;
  std::mt19937 engine(rd());
  std::uniform_real_distribution<double> normal_distrib(0.0, 1.0);

  return normal_distrib(engine);
}

// simple min-max normalization
std::vector<std::vector<double>> NormalizedGraph(const Graph& graph) {
  const std::size_t sz = graph.Size();
  const double min = graph.MinWeight();
  const double max = graph.MaxWeight();
  
  std::vector<std::vector<double>> normalized(sz, std::vector<double>(sz));

  for(std::size_t i = 0; i != sz; ++i)
    for(std::size_t j = 0; j != sz; ++j)
      normalized[i][j] = (graph[i][j] - min) / (max - min);

  return normalized;
}

int Roulette(const std::vector<double>& chance, const std::vector<bool>& visited) {
  int next_point;
  std::size_t visited_num = std::count(visited.begin(), visited.end(), true);
  std::vector<std::pair<double, double>> intervals(chance.size() - visited_num);

  std::cout << "intervals size = " << intervals.size() << std::endl;
  for (std::size_t i = 0, j = 0; i < chance.size(); ++i) {
    if (chance[i] == 0) continue;

    double first = (j == 0) ? 0 : intervals[j - 1].second;
    double second = first + chance[i];

    intervals[j++] = (std::make_pair(first, second));
  }
  
  for (auto pair : intervals)
    std::cout << pair.first << "-" << pair.second << std::endl;

  const double random_value = RandomValue();
  std::cout << "RANDOMMED " << random_value << std::endl;

  for (std::size_t i = 0, j = 0; i != chance.size(); ++i) {
    if (chance[i] == 0) continue;
    if (random_value > intervals[j].first && random_value <= intervals[j++].second)
      next_point = i;
  }

  std::cout << "next point will be " << next_point << "\n\n";
  return next_point;
}

GraphAlgorithms::TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(const Graph &graph) {
  TsmResult res;
  const int sz = graph.Size();
  /* const double Q = 4.0; */

  const double alpha = 1.0;
  const double beta = 0.0;

  // матрица близости
  std::vector<std::vector<double>> dist = NormalizedGraph(graph);
  // матрица меток ( феромонов )
  std::vector<std::vector<double>> fero(sz, std::vector<double>(sz, 0.2));

  /* std::cout << "Distances: \n"; */
  /* for (int i = 0; i < sz; ++i) { */
  /*   for (int j = 0; j < sz; ++j) */
  /*     std::cout << dist[i][j] << " "; */
  /*   std::cout << std::endl; */
  /* } */
  /* std::cout << std::endl; */

  // проход муравья от каждой вершины по всем остальным
  int start_point = 0;
  std::vector<bool> visited(sz, false);
  for (int i = 0; i < sz; ++i) {

    visited[start_point] = true;
    // матрица желаний перейти из города i в город j;
    std::vector<double> wish(sz);
    for (int j = 0; j != sz; ++j) {
      wish[j] = !visited[j] * std::pow(fero[start_point][j], alpha) *
                             std::pow(dist[start_point][j], beta);
    }

    std::cout << "wishes: ";
    for (auto w : wish)
      std::cout << w << " ";
    std::cout << std::endl;

    double wish_sum = std::accumulate(wish.begin(), wish.end(), 0.0);
    // матрица вероятностей перейти от города i в город j
    std::vector<double> chance(sz);
    for (int j = 0; j != sz; ++j) {
      chance[j] = wish[j] / wish_sum;
    }

    std::cout << "chances: ";
    for (auto p : chance)
      std::cout << p << " ";
    std::cout << std::endl;

    // тут надо выбрать в какой город идти дальше
    // рулетка должна работать только для непосещенных еще городов и их вероятностей

    std::cout << "Sum of all chances = " << std::accumulate(chance.begin(), chance.end(), 0.0);
    std::cout << "\n\n";

    start_point = Roulette(chance, visited);


    /* changing start point for ant */

    break;

  }

  return res;
}
