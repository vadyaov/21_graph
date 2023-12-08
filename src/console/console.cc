#include "console.h"

void Console::Run() {
  std::cout << "---------------------------------------------------------\n";
  std::cout << "                        S21_GRAPH                        \n";
  std::cout << "---------------------------------------------------------\n";

  std::cout << "1. Load graph from file\n"
               "2. BFS\n"
               "3. DFS\n"
               "4. Djkstra\n"
               "5. Floyd-Warshall\n"
               "6. Prim\n"
               "7. ACO\n";
  int act = -1;
  std::cin >> act;

  while (act != Action::EXIT) {

    if (act == Action::LOAD) {

      std::cout << "Enter the file path:\n";
      std::string path;
      std::cin >> path;
      try {
        g.LoadGraphFromFile(path);
        std::cout << "SUCCESS!\n";
      } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
      }

    } else if (!g.Empty()) {

      if (act == Action::BREADTH) {

        std::cout << "Start from: ";
        int start;
        std::cin >> start;
        try {
          std::vector<int> bfs_result = GraphAlgorithms::BreadthFirstSearch(g, start);
          for (std::size_t i = 0; i != bfs_result.size(); ++i) {
            std::cout << bfs_result[i];
            if (i == bfs_result.size() - 1)
              std::cout << "\n";
            else
              std::cout << "-->";
          }
          std::cout << "\n";
        } catch (const std::exception& e) {
          std::cout << e.what() << std::endl;
        }

      } else if (act == Action::DEPTH) {

        std::cout << "Start from: ";
        int start;
        std::cin >> start;
        try {
          std::vector<int> bfs_result = GraphAlgorithms::DepthFirstSearch(g, start);
          for (std::size_t i = 0; i != bfs_result.size(); ++i) {
            std::cout << bfs_result[i];
            if (i == bfs_result.size() - 1)
              std::cout << "\n";
            else
              std::cout << "-->";
          }
          std::cout << "\n";
        } catch (const std::exception& e) {
          std::cout << e.what() << std::endl;
        }

      } else if (act == Action::DIJKSTRA) {
      } else if (act == Action::FL_WARSH) {
      } else if (act == Action::PRIM) {
      } else if (act == Action::ACO) {
      } else {
        // something else
      }
    }

    std::cin >> act;

  }


}
