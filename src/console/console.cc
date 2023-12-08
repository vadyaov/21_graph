#include "console.h"

#include <ncurses.h>

void print_menu(WINDOW *menu_win, int highlight, const std::vector<std::string>& choices) {
	int x = 2, y = 2;	

	box(menu_win, 0, 0);
	for(std::size_t i = 0; i < choices.size(); ++i) {
    if(highlight == (int)(i + 1)) {
      wattron(menu_win, A_REVERSE); 
			mvwprintw(menu_win, y, x, "%s", choices[i].data());
			wattroff(menu_win, A_REVERSE);
		} else
			mvwprintw(menu_win, y, x, "%s", choices[i].data());
		++y;
	}
	wrefresh(menu_win);
}

void Console::Run() {
  WINDOW *menu_win;

  const std::vector<std::string> choices = { 
        "Loading graph from .txt",
        "Breadth First Search (BFS)",
        "Depth First Search (DFS)",
        "Shortest path between two points (Dijkstra)",
        "Shortest paths between all points (Floyd-Warshall)",
        "Minimum Spanning Tree (Prim)",
        "Travelling Salesman Problem (Ant Colony Optimization)",
        "Exit"
  };

	int highlight = 1;
	int choice = 0;

	initscr();
	clear();
  echo();
	/* noecho(); */
	cbreak();	/* Line buffering disabled. pass on everything */
  curs_set(0);

	int startx = (100 - width) / 2;
	int starty = (15 - heigh) / 2;
		
	menu_win = newwin(heigh, width, starty, startx);
	keypad(menu_win, TRUE);
	mvprintw(0, 5, "Use arrow keys to go up and down, Press enter to select a choice");
	refresh();

	print_menu(menu_win, highlight, choices);
	while(1) {
    int c = wgetch(menu_win);
		switch(c) {
      case KEY_UP:
				if(highlight == 1)
					highlight = choices.size();
				else
					--highlight;
				break;
			case KEY_DOWN:
				if(highlight == (int)choices.size())
					highlight = 1;
				else 
					++highlight;
				break;
			case 10:
				choice = highlight;
				break;
			/* default: */
			/* 	mvprintw(24, 0, "Charcter pressed is = %3d Hopefully it can be printed as '%c'", c, c); */
			/* 	refresh(); */
			/* 	break; */
		}
		print_menu(menu_win, highlight, choices);

    if (choice != 0) {
      /* echo(); */
      mvprintw(15, 0, "%d. %s\n", choice, choices[choice - 1].data());
      switch (choice) {
        case Action::LOAD: {
          mvprintw(16, 1, "Enter path to file: ");
          char path[128];
          /* echo(); */
          getstr(path);
          /* noecho(); */
          clrtoeol();
          try {
            g.LoadGraphFromFile(path);
            mvprintw(17, 0, "Success");
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
      } case Action::BFS: {
          mvprintw(16, 1, "Enter the start point: ");
          int start;
          scanw("%d", &start);
          try {
            std::vector<int> result = GraphAlgorithms::BreadthFirstSearch(g, start);
            for (std::size_t i = 0, j = 1; i != result.size(); ++i, j += 3) {
              mvprintw(17, j, "%d", result[i]);
              refresh();
            }
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
      } case Action::DFS: {
          mvprintw(16, 1, "Enter the start point: ");
          int start;
          scanw("%d", &start);
          try {
            std::vector<int> result = GraphAlgorithms::DepthFirstSearch(g, start);
            for (std::size_t i = 0, j = 1; i != result.size(); ++i, j += 3) {
              mvprintw(17, j, "%d", result[i]);
              refresh();
            }
          } catch (const std::exception& e) {
            mvprintw(16, 0, "%s\n", e.what());
          }
          break;
      } case Action::DJK: {

          break;
      } case Action::FL_WRSH: {

          break;
      } case Action::PRIM: {

          break;
      } case Action::ACO:

          break;
      }
      refresh();
    }

    move(16, 0);
    clrtoeol();

    move(17, 0);
    clrtoeol();

		if (choice == static_cast<int>(choices.size()))	/* User did a choice come out of the infinite loop */
			break;
    choice = 0;
	}	
	mvprintw(25, 0, "PRESS ANY KEY TO QUIT");
	refresh();
  getch();
	clrtoeol();
	endwin();
}
