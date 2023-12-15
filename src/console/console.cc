#include "console.h"

#include <ncurses.h>

void print_menu(WINDOW* menu_win, int highlight,
                const std::vector<std::string>& choices) {
  int x = 2, y = 2;

  box(menu_win, 0, 0);
  for (std::size_t i = 0; i < choices.size(); ++i) {
    if (highlight == (int)(i + 1)) {
      wattron(menu_win, A_REVERSE);
      mvwprintw(menu_win, y, x, "%s", choices[i].data());
      wattroff(menu_win, A_REVERSE);
    } else
      mvwprintw(menu_win, y, x, "%s", choices[i].data());
    ++y;
  }
  wrefresh(menu_win);
}

void print_graph(WINDOW* graph_win, const Graph& g) {
  int x = 1, y = 1;
  box(graph_win, 0, 0);
  for (std::size_t i = 0, sz = g.Size(); i != sz; ++i) {
    for (std::size_t j = 0; j != sz; ++j) {
      mvwprintw(graph_win, y, x, "%d", g[i][j]);
      x += 4;
    }
    y++;
    x = 1;
  }
  wrefresh(graph_win);
}

void Console::Run() {
  WINDOW* menu_win = nullptr;
  WINDOW* graph_win = nullptr;
  WINDOW* fl_wrsh_win = nullptr;

  const std::vector<std::string> choices = {
      "Loading graph from .txt",
      "Breadth First Search (BFS)",
      "Depth First Search (DFS)",
      "Shortest path between two points (Dijkstra)",
      "Shortest paths between all points (Floyd-Warshall)",
      "Minimum Spanning Tree (Prim)",
      "Travelling Salesman Problem (Ant Colony Optimization)",
      "Exit"};

  int highlight = 1;
  int choice = 0;

  initscr();
  clear();
  cbreak(); /* Line buffering disabled. pass on everything */
  curs_set(0);

  int startx = (100 - width) / 2;
  int starty = (15 - heigh) / 2;

  menu_win = newwin(heigh, width, starty, startx);
  keypad(menu_win, TRUE);

  print_menu(menu_win, highlight, choices);
  while (1) {
    noecho();
    int c = wgetch(menu_win);
    switch (c) {
      case KEY_UP:
        if (highlight == 1)
          highlight = choices.size();
        else
          --highlight;
        break;
      case KEY_DOWN:
        if (highlight == (int)choices.size())
          highlight = 1;
        else
          ++highlight;
        break;
      case 10:
        choice = highlight;
        break;
    }

    print_menu(menu_win, highlight, choices);

    if (choice != 0) {
      echo();
      if (fl_wrsh_win) {
        wclear(fl_wrsh_win);
        wrefresh(fl_wrsh_win);
      }

      switch (choice) {
        case Action::LOAD: {
          mvprintw(16, 1, "Enter path to file: ");
          char path[128];
          getstr(path);
          clrtoeol();
          try {
            g.LoadGraphFromFile(path);
            mvprintw(17, 0, "Success");

            if (graph_win) {
              wclear(graph_win);
              wrefresh(graph_win);
            }

            graph_win = newwin(g.Size() + 2, g.Size() * 4, 40, 30);
            print_graph(graph_win, g);
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
        }
        case Action::BFS: {
          mvprintw(16, 1, "Enter the start point: ");
          int start;
          scanw("%d", &start);
          try {
            std::vector<int> result =
                GraphAlgorithms::BreadthFirstSearch(g, start);
            for (std::size_t i = 0, j = 1; i != result.size(); ++i, j += 3) {
              mvprintw(17, j, "%d", result[i]);
              refresh();
            }
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
        }
        case Action::DFS: {
          mvprintw(16, 1, "Enter the start point: ");
          int start;
          scanw("%d", &start);
          try {
            std::vector<int> result =
                GraphAlgorithms::DepthFirstSearch(g, start);
            for (std::size_t i = 0, j = 1; i != result.size(); ++i, j += 3) {
              mvprintw(17, j, "%d", result[i]);
              refresh();
            }
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
        }
        case Action::DJK: {
          mvprintw(16, 1, "Enter the start/finish points: ");
          int start = 0, finish = 0;
          scanw("%d %d", &start, &finish);
          try {
            mvprintw(17, 1, "%d",
                     GraphAlgorithms::GetShortestPathBetweenVertices(g, start,
                                                                     finish));
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
        }
        case Action::FL_WRSH: {
          try {
            auto res = GraphAlgorithms::GetShortestPathsBetweenAllVertices(g);
            fl_wrsh_win = newwin(g.Size() + 2, g.Size() * 4, 19, 1);
            int x = 1, y = 1;
            box(fl_wrsh_win, 0, 0);
            for (std::size_t i = 0, sz = res.size(); i != sz; ++i) {
              for (std::size_t j = 0; j != sz; ++j) {
                mvwprintw(fl_wrsh_win, y, x, "%d", res[i][j]);
                x += 4;
              }
              y++;
              x = 1;
            }
            wrefresh(fl_wrsh_win);
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
        }
        case Action::PRIM: {
          try {
            auto res = GraphAlgorithms::GetLeastSpanningTree(g);
            fl_wrsh_win = newwin(g.Size() + 2, g.Size() * 4, 19, 1);
            int x = 1, y = 1;
            box(fl_wrsh_win, 0, 0);
            for (std::size_t i = 0, sz = res.size(); i != sz; ++i) {
              for (std::size_t j = 0; j != sz; ++j) {
                mvwprintw(fl_wrsh_win, y, x, "%d", res[i][j]);
                x += 4;
              }
              y++;
              x = 1;
            }
            wrefresh(fl_wrsh_win);
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
        }
        case Action::ACO:
          try {
            GraphAlgorithms::TsmResult res =
                GraphAlgorithms::SolveTravelingSalesmanProblem(g);
            for (std::size_t i = 0, j = 1, sz = res.vertices.size(); i != sz;
                 ++i, j += 3) {
              mvprintw(17, j, "%d", res.vertices[i]);
            }
            mvprintw(18, 1, "distance = %lf", res.distance);
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
      }
      refresh();
    }

    for (int i = 16; i != 19; ++i) {
      move(i, 0);
      clrtoeol();
    }

    if (choice == static_cast<int>(choices.size())) break;

    choice = 0;
  }

  mvprintw(55, 0, "PRESS ANY KEY TO QUIT");
  refresh();
  getch();
  clrtoeol();
  endwin();
}
