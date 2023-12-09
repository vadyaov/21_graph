#ifndef S21_GRAPH_H_
#define S21_GRAPH_H_

#include <vector>
#include <string>
#include <fstream>

class Graph {
  private:
    struct ProxyRow {
      ProxyRow(int* r) : row{r} {};
      ProxyRow(const int* r) : row{const_cast<int*>(r)} {};

      int *row;
      int& operator[](int n) { return row[n]; }
      const int& operator[](int n) const { return row[n]; }
    };

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

    /* double MinWeight() const noexcept; */
    /* double MaxWeight() const noexcept; */

  public:
    friend std::ostream& operator<<(std::ostream& os, const Graph& g);

    // just for fun, allows to do graph[i][j] :)
    ProxyRow operator[](int row) {
      return adjacent_.data() + row * size; // implicit cast to ProxyRow(int *)
    }

    const ProxyRow operator[](int row) const {
      return adjacent_.data() + row * size; // implicit cast to ProxyRow(int *)
    }

  private:
    // only directed or undirected graphs
    bool Directed() const noexcept;

  private:
    std::vector<int> adjacent_;  // adjacent matrix
    std::size_t size;            // vertex number
    bool directed;
};


#endif // S21_GRAPH_H_
