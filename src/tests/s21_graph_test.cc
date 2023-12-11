#include <gtest/gtest.h>
#include <filesystem>

#include "../graph/s21_graph.h"

TEST(load_export, t1) {
  const std::string path {"../examples/graph_0.txt"};
  Graph g;

  ASSERT_EQ(g.Empty(), true);

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);
  g.ExportGraphToDot("../examples/dots/graph_0.dot");

  std::filesystem::current_path(currentWorkingDirectory);

  ASSERT_EQ(g.Size(), 11);
  EXPECT_EQ(g.IsDirect(), false);

  const std::vector<std::vector<int>> expected = {
    {0, 29, 20, 21, 16, 31, 100, 12, 4, 31, 18},
    {29, 0, 15, 29, 28, 40, 72, 21, 29, 41, 12},
    {20, 15, 0, 15, 14, 25, 81, 9, 23, 27, 13},
    {21, 29, 15, 0, 4, 12, 92, 12, 25, 13, 25},
    {16, 28, 14, 4, 0, 16, 94, 9, 20, 16, 22},
    {31, 40, 25, 12, 16, 0, 95, 24, 36, 3, 37},
    {100, 72, 81, 92, 94, 95, 0, 90, 101, 99, 84},
    {12, 21, 9, 12, 9, 24, 90, 0, 15, 25, 13},
    {4, 29, 23, 25, 20, 36, 101, 15, 0, 35, 18},
    {31, 41, 27, 13, 16, 3, 99, 25, 35, 0, 38},
    {18, 12, 13, 25, 22, 37, 84, 13, 18, 38, 0}};

  for (std::size_t i = 0, sz = g.Size(); i != sz; ++i)
    for (std::size_t j = 0; j != sz; ++j)
      EXPECT_EQ(g[i][j], expected[i][j]);

}

TEST(load_export, t2) {
  const std::string path {"../examples/graph_1.txt"};
  Graph g;

  ASSERT_EQ(g.Empty(), true);

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);
  g.ExportGraphToDot("../examples/dots/graph_1.dot");

  std::filesystem::current_path(currentWorkingDirectory);

  ASSERT_EQ(g.Size(), 8);
  EXPECT_EQ(g.IsDirect(), false);

  const std::vector<std::vector<int>> expected = {
    {0, 1, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0},
    {0, 1, 0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 1, 0, 0, 0},
    {0, 0, 0, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 1, 1, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 0}};

  for (std::size_t i = 0, sz = g.Size(); i != sz; ++i)
    for (std::size_t j = 0; j != sz; ++j)
      EXPECT_EQ(g[i][j], expected[i][j]);
}

TEST(load_export, t3) {
  const std::string path {"../examples/graph_2.txt"};
  Graph g;

  ASSERT_EQ(g.Empty(), true);

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);
  g.ExportGraphToDot("../examples/dots/graph_2.dot");

  std::filesystem::current_path(currentWorkingDirectory);

  ASSERT_EQ(g.Size(), 8);
  EXPECT_EQ(g.IsDirect(), true);

  const std::vector<std::vector<int>> expected = {
    {0, 1, 0, 0, 0, 0, 0, 1},
    {0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}};

  for (std::size_t i = 0, sz = g.Size(); i != sz; ++i)
    for (std::size_t j = 0; j != sz; ++j)
      EXPECT_EQ(g[i][j], expected[i][j]);
}

TEST(load_export, t4) {
  const std::string path {"../examples/corm_graph.txt"};
  Graph g;

  ASSERT_EQ(g.Empty(), true);

  std::string currentWorkingDirectory = std::filesystem::current_path().string();

  std::filesystem::current_path("/home/vadim/Projects/School21/21_graph/src/tests");

  g.LoadGraphFromFile(path);
  g.ExportGraphToDot("../examples/dots/corm_graph.dot");

  std::filesystem::current_path(currentWorkingDirectory);

  ASSERT_EQ(g.Size(), 6);
  EXPECT_EQ(g.IsDirect(), true);

  const std::vector<std::vector<int>> expected = {
    {0, 1, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 1, 1},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 1}};

  for (std::size_t i = 0, sz = g.Size(); i != sz; ++i)
    for (std::size_t j = 0; j != sz; ++j)
      EXPECT_EQ(g[i][j], expected[i][j]);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
