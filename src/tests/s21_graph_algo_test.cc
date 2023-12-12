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

  EXPECT_EQ(bfs, expected);
}

TEST(algo_bfs, t3) {
  const std::string path {"../examples/bfs_graph.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto bfs = GraphAlgorithms::BreadthFirstSearch(g, 2);

  std::vector<int> expected {2, 3, 4, 5, 1, 6};

  EXPECT_EQ(bfs, expected);
}

TEST(algo_bfs, t4) {
  const std::string path {"../examples/bfs_graph.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto bfs = GraphAlgorithms::BreadthFirstSearch(g, 1);

  std::vector<int> expected {1, 2, 3, 4, 5, 6};

  EXPECT_EQ(bfs, expected);
}

TEST(algo_dfs, t1) {
  const std::string path {"../examples/bfs_graph.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto dfs = GraphAlgorithms::DepthFirstSearch(g, 1);

  std::vector<int> expected {1, 3, 4, 5, 6, 2};

  EXPECT_EQ(dfs, expected);
}

TEST(algo_dfs, t2) {
  const std::string path {"../examples/bfs_graph.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto dfs = GraphAlgorithms::DepthFirstSearch(g, 4);

  std::vector<int> expected {4, 5, 6, 2, 3, 1};

  EXPECT_EQ(dfs, expected);
}

TEST(algo_dfs, t3) {
  const std::string path {"../examples/corm_graph.txt"};
  Graph g;


  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto dfs = GraphAlgorithms::DepthFirstSearch(g, 2);

  std::vector<int> expected {2, 5, 4};

  EXPECT_EQ(dfs, expected);
}

TEST(algo_dfs, t4) {
  const std::string path {"../examples/corm_graph.txt"};
  Graph g;

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto bfs = GraphAlgorithms::DepthFirstSearch(g, 3);

  std::vector<int> expected {3, 6, 5, 4, 2};

  EXPECT_EQ(bfs, expected);
}

TEST(algo_dijkstra, t1) {
  const std::string path {"../examples/dijkstra_graph.txt"};
  Graph g;

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  int dist = GraphAlgorithms::GetShortestPathBetweenVertices(g, 1, 4);

  EXPECT_EQ(dist, 9);
}

TEST(algo_dijkstra, t2) {
  const std::string path {"../examples/dijkstra_graph.txt"};
  Graph g;

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  int dist15 = GraphAlgorithms::GetShortestPathBetweenVertices(g, 1, 5);
  int dist13 = GraphAlgorithms::GetShortestPathBetweenVertices(g, 1, 3);
  int dist12 = GraphAlgorithms::GetShortestPathBetweenVertices(g, 1, 2);

  EXPECT_EQ(dist15, 7);
  EXPECT_EQ(dist13, 5);
  EXPECT_EQ(dist12, 8);
}

TEST(algo_floyd_warsh, t1) {
  const std::string path {"../examples/floyd_warsh_.txt"};
  Graph g;

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto dist = GraphAlgorithms::GetShortestPathsBetweenAllVertices(g);

  std::vector<std::vector<int>> expected {
      {0, 1, -3, 2, -4},
      {3, 0, -4, 1, -1},
      {7, 4, 0, 5, 3},
      {2, -1, -5, 0, -2},
      {8, 5, 1, 6, 0}
  };

  EXPECT_EQ(dist, expected);

}

TEST(algo_floyd_warsh, t2) {
  const std::string path {"../examples/fl_wrsh_1.txt"};
  Graph g;

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);

  std::filesystem::current_path(currentWorkingDirectory);

  auto dist = GraphAlgorithms::GetShortestPathsBetweenAllVertices(g);

  std::vector<std::vector<int>> expected {
      {0, 3, 7, 5},
      {2, 0, 6, 4},
      {3, 1, 0, 5},
      {5, 3, 2, 0}
  };

  EXPECT_EQ(dist, expected);

}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
