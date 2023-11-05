#include "s21_graph.h"

#include <iomanip>
#include <cmath>

void Graph::LoadGraphFromFile(const std::string& filename) {
  std::ifstream istrm;
  istrm.open(filename);

  if (!istrm.is_open())
    throw std::invalid_argument("Can't open file.");

  istrm >> size;

  adjacent_.resize(size * size);

  for (std::size_t i = 0; i < adjacent_.size(); ++i) {
    istrm >> adjacent_[i];
  }

  istrm.close();
}

std::ostream& operator<<(std::ostream& os, const Graph& g) {
  for (std::size_t i = 0, j = 1; i < g.size * g.size; ++i, ++j) {
    os << std::setw(4);
    os << g.adjacent_[i];
    if (j == g.size) {
      os << std::endl;
      j = 0;
    }
  }

  return os;
}
