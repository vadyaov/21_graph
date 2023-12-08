#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <iostream>
/* #include <ncurces.h> */

#include "../graph/s21_graph.h"
#include "../algoritms/s21_graph_algorithms.h"

class Console {
  public:
    enum Action {EXIT = 0, LOAD, BREADTH, DEPTH, DIJKSTRA, FL_WARSH, PRIM, ACO};

    void Run();

  private:
    Graph g;
};

#endif // CONSOLE_H_
