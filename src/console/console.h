#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <iostream>
/* #include <ncurces.h> */

#include "../algorithms/s21_graph_algorithms.h"
#include "../graph/s21_graph.h"

class Console {
 public:
  enum Action { LOAD = 1, BFS, DFS, DJK, FL_WRSH, PRIM, ACO };

  void Run();

 private:
  Graph g;
  int width = 60;
  int heigh = 12;
};

#endif  // CONSOLE_H_
