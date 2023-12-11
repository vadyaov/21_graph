#include <gtest/gtest.h>
#include <filesystem>

#include "../algorithms/s21_graph_algorithms.h"

TEST(algo_bfs, t1) {
  const std::string path {"../examples/graph_0.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto bfs = GraphAlgorithms::BreadthFirstSearch(g, 1);

  for (std::size_t i = 0; i != bfs.size(); ++i)
    EXPECT_EQ(bfs[i], i + 1);

}

TEST(algo_bfs, t2) {
  const std::string path {"../examples/graph_1.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto bfs = GraphAlgorithms::BreadthFirstSearch(g, 1);

  std::vector<int> expected {1, 2, 8, 3, 4, 5, 6, 7};

  ASSERT_EQ(bfs.size(), expected.size());
  for (std::size_t i = 0; i != bfs.size(); ++i)
    EXPECT_EQ(bfs[i], expected[i]);

}
