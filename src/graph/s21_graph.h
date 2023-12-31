#ifndef S21_GRAPH_H_
#define S21_GRAPH_H_

#include <fstream>
#include <string>
#include <vector>

class Graph {
 private:
  struct ProxyRow {
    ProxyRow(int* r) : row{r} {};
    ProxyRow(const int* r) : row{const_cast<int*>(r)} {};

    int* row;
    int& operator[](int n) { return row[n]; }
    const int& operator[](int n) const { return row[n]; }
  };

  bool Directed() const noexcept;

 public:
  Graph() : adjacent_{}, size{0}, directed{false} {}

  // loading graph from file with adjacent matrix
  void LoadGraphFromFile(const std::string& filename);

  // exporting graph to .gv (.dot) file
  void ExportGraphToDot(const std::string& filename) const;

  // what if called on empty graph ?
  bool IsDirect() const noexcept;

  bool Empty() const noexcept;

  std::size_t Size() const noexcept;

 public:
  /* friend std::ostream& operator<<(std::ostream& os, const Graph& g); */

  /* ProxyRow operator[](int row) { */
  /*   return adjacent_.data() + row * size; */
  /* } */

  const ProxyRow operator[](int row) const {
    return adjacent_.data() + row * size;
  }

 private:
  std::vector<int> adjacent_;  // adjacent matrix
  std::size_t size;            // vertex number
  bool directed;
};

#endif  // S21_GRAPH_H_
