#ifndef S21_GRAPH_ALGORITHMS_H_
#define S21_GRAPH_ALGORITHMS_H_

#include "../graph/s21_graph.h"

class GraphAlgorithms {
  public:
    struct TsmResult {
      std::vector<int> vertices;
      double distance{0};
    };

    // usign stack
    static std::vector<int> DepthFirstSearch(const Graph& graph, int s);

    // usign queue
    static std::vector<int> BreadthFirstSearch(const Graph& graph, int s);

    // Dijkstra algo
    static int GetShortestPathBetweenVertices(const Graph& graph, int v1, int v2);

    // Floyd-Warshall algo O(V^3)
    static std::vector<std::vector<int>>
      GetShortestPathsBetweenAllVertices(const Graph& graph);

    static std::vector<std::vector<int>> GetLeastSpanningTree(const Graph& graph);

    static TsmResult SolveTravelingSalesmanProblem(const Graph &graph);
};

#endif // S21_GRAPH_ALGORITHMS_H_
