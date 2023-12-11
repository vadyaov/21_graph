cmake_minimum_required(VERSION 3.14)
project(tests)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

link_directories(./../)

# Граф
add_executable(
  graph_test
  s21_graph_test.cc
)
target_link_libraries(
  graph_test
  GTest::gtest_main
  -l:s21_graph.a
)

include(GoogleTest)
gtest_discover_tests(graph_test)

# Алгоритмы
add_executable(
  algo_test
  s21_graph_algo_test.cc
)
target_link_libraries(
  algo_test
  GTest::gtest_main
  -l:s21_graph.a
  -l:s21_graph_algorithms.a
)

include(GoogleTest)
gtest_discover_tests(algo_test)
