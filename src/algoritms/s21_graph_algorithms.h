#ifndef S21_GRAPH_ALGORITHMS_H_
#define S21_GRAPH_ALGORITHMS_H_

#include "../graph/s21_graph.h"

class GraphAlgorithms {
  public:

    enum Color {WHITE, GRAY, BLACK};

    // usign stack
    static std::vector<int> DepthFirstSearch(const Graph &graph, int s);

    // usign queue
    static std::vector<int> BreadthFirstSearch(const Graph &graph, int s);
};

#endif // S21_GRAPH_ALGORITHMS_H_
