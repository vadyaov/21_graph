#include "s21_graph.h"

#include <iomanip>
#include <iostream>

void Graph::LoadGraphFromFile(const std::string& filename) {
  std::ifstream istrm;
  istrm.open(filename, std::ios_base::in);

  if (!istrm.is_open())
    throw std::invalid_argument("Can't open file.");

  istrm >> size;

  adjacent_.resize(size * size);

  for (std::size_t i = 0; i < adjacent_.size(); ++i) {
    istrm >> adjacent_[i];
  }

  istrm.close();
}

void Graph::ExportGraphToDot(const std::string& filename) const {
  if (adjacent_.empty()) return;

  std::ofstream ostrm;
  ostrm.open(filename, std::ios::out);

  const bool directed = Directed();
  const std::string tab(4, ' ');
  const std::string connection = directed ? "->" : "--";
  std::string graph_title = (directed ? "digraph " : "graph ") +
    filename.substr(0, filename.find_first_of('.'));

  /* what if loops ? */

  ostrm << graph_title << " {\n";

  ostrm << tab << "here the main logic\n";

  ostrm << "}\n";

  ostrm.close();
}

bool Graph::Directed() const noexcept {

  for (std::size_t i = 0; i < size; ++i) {
    for (std::size_t j = 0; j < size; ++j) {
      if (i == j) continue; // think about this: what if [i][j] != 0 ? --> loop

      // only allows to detect directed and undirected graphs (not mixed)
      if (adjacent_[i * size + j] != adjacent_[j * size + i])
        return true;

    }
  }

  return false;
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
