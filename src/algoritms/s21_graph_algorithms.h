#ifndef S21_GRAPH_ALGORITHMS_H_
#define S21_GRAPH_ALGORITHMS_H_

#include "../graph/s21_graph.h"

class GraphAlgorithms {
  public:

    // usign stack
    static std::vector<int> DepthFirstSearch(const Graph &graph, int s);

    // usign queue
    static std::vector<int> BreadthFirstSearch(const Graph &graph, int s);

    static std::size_t GetShortestPathBetweenVertices(const Graph &graph,
                                                                int v1, int v2);
};

#endif // S21_GRAPH_ALGORITHMS_H_
