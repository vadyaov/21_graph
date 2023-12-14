#include "s21_graph.h"

#include <algorithm>

void Graph::LoadGraphFromFile(const std::string& filename) {
  std::ifstream istrm;
  istrm.open(filename, std::ios_base::in);

  if (!istrm.is_open())
    throw std::invalid_argument("Can not open file " + filename);

  istrm >> size;

  adjacent_.resize(size * size);

  for (std::size_t i = 0; i < adjacent_.size(); ++i) {
    istrm >> adjacent_[i];
  }

  directed = Directed();

  istrm.close();
}

std::string GetName(const std::string& filename) {
  std::string name;
  std::string::const_iterator it = filename.begin() + filename.find_last_of('.') - 1;
  if (it < filename.begin()) throw std::invalid_argument("Incorrect file extension");

  for (;it >= filename.cbegin() && *it != '/'; --it) {
    name.insert(name.begin(), *it);
  }

  return name;
}

void Graph::ExportGraphToDot(const std::string& filename) const {
  if (adjacent_.empty()) return;

  std::ofstream ostrm;
  ostrm.open(filename, std::ios::out);

  if (!ostrm.is_open())
    throw std::invalid_argument("Can not open file " + filename);

  const std::string tab(4, ' ');
  const std::string connection = directed ? " -> " : " -- ";

  std::string name = GetName(filename);

  std::string graph_title = (directed ? "digraph " : "graph ") + name;

  ostrm << graph_title << " {\n";

  std::size_t last = 0;
  for (std::size_t i = 0; i < size; ++i) {
    for (std::size_t j = directed ? 0 : i; j < size; ++j) {
      if (adjacent_[i * size + j] != 0) {
        if (i + 1 == last) {
          ostrm << connection << j + 1;
        } else {
          if (last) ostrm << std::endl;
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
