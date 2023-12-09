#include "s21_graph.h"

#include <iomanip>
#include <iostream>
#include <algorithm>

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

  directed = Directed();

  istrm.close();
}

void Graph::ExportGraphToDot(const std::string& filename) const {
  if (adjacent_.empty()) return;

  std::ofstream ostrm;
  ostrm.open(filename, std::ios::out);

  const std::string tab(4, ' ');
  const std::string connection = directed ? " -> " : " -- ";
  std::string graph_title = (directed ? "digraph " : "graph ") +
    filename.substr(0, filename.find_first_of('.'));

  ostrm << graph_title << " {\n";

  std::size_t last = 0;
  for (std::size_t i = 0; i < size; ++i) {
    for (std::size_t j = directed ? 0 : i; j < size; ++j) {
      if (adjacent_[i * size + j] != 0) {
        if (i + 1 == last) {
          ostrm << connection << j + 1;
        } else {
          if (last) // to not put \n before the first row ('last' value is still zero)
            ostrm << std::endl;
          ostrm << tab << i + 1 << connection << j + 1;
        }

        last = j + 1;
      }
    }
  }

  ostrm << "\n}\n";

  ostrm.close();
}

bool Graph::Directed() const noexcept {

  for (std::size_t i = 0; i < size; ++i) {
    for (std::size_t j = 0; j < size; ++j) {

        // only allows to detect directed and undirected graphs (not mixed)
      if (i != j && adjacent_[i * size + j] != adjacent_[j * size + i])
        return true;
    }
  }

  return false;
}

bool Graph::IsDirect() const noexcept {
  return directed;
}

bool Graph::Empty() const noexcept {
  return size == 0 || adjacent_.empty();
}

std::size_t Graph::Size() const noexcept {
  return size;
}

/* double Graph::MinWeight() const noexcept { */
/*   return *std::min_element(adjacent_.begin(), adjacent_.end()); */
/* } */

/* double Graph::MaxWeight() const noexcept { */
/*   return *std::max_element(adjacent_.begin(), adjacent_.end()); */
/* } */

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
