#ifndef S21_GRAPH_H_
#define S21_GRAPH_H_

#include <vector>
#include <string>
#include <fstream>

class Graph {
  private:
    struct ProxyRow {
      ProxyRow(int* r) : row{r} {};

      int *row;
      int& operator[](int n) { return row[n]; }
      /* const int& operator[](int n) const { return row[n]; } */
    };

  public:
    Graph() : adjacent_{}, size{0} {}

    void LoadGraphFromFile(const std::string& filename);
    /* void ExportGraphToDot(const std::string& filename) const; */





    friend std::ostream& operator<<(std::ostream& os, const Graph& g);

    // just for fun, allows to do graph[i][j] :)
    ProxyRow operator[](int row) {
      return adjacent_.data() + row * size; 
    }

  private:
    std::vector<int> adjacent_;  // adjacent matrix, mb better use S21Maztrix class ?
    std::size_t size;            // sqrt
};


#endif // S21_GRAPH_H_
