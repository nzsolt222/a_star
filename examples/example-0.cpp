#include "astar.hpp"

#include <iostream>
#include <vector>
#include <iterator>

/**
 * @brief Print a 2D container.
 *
 * @tparam Grid container type
 * @param grid a container
 */
template <typename Grid>
void print_grid(const Grid &grid)
{
    for (const auto &row : grid)
    {
        for (const auto &element : row)
        {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}

int main(int, char const *[])
{
    // create a grid map
    // or std::vector<std::vector<char>>
    char map[12][9] = {{'+', '-', '-', '-', '-', '-', '-', '-', '+'},
                       {'|', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '|'},
                       {'|', ' ', '#', ' ', '#', '#', '#', '#', '|'},
                       {'|', ' ', ' ', '#', '#', '#', '#', ' ', '|'},
                       {'|', ' ', ' ', ' ', '#', ' ', ' ', ' ', '|'},
                       {'|', ' ', ' ', ' ', '#', '*', '#', ' ', '|'},
                       {'|', ' ', '#', '#', '#', ' ', ' ', ' ', '|'},
                       {'|', ' ', ' ', ' ', '#', ' ', ' ', ' ', '|'},
                       {'|', ' ', ' ', ' ', '#', '#', ' ', ' ', '|'},
                       {'|', ' ', ' ', '#', ' ', ' ', ' ', ' ', '|'},
                       {'|', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '|'},
                       {'+', '-', '-', '-', '-', '-', '-', '-', '+'}};

    // create an astar node with proper width and height
    astar::Grid grid = astar::Grid(12, 9, astar::Straight());

    // set witch item is accesable
    grid.set_accesable([&](int x1, int y1, int, int)
                       {
                           return map[x1][y1] == ' ';
                       });

    // find the path
    auto solution = grid.find(1, 1, 5, 5);  // std::vector<std::pair<int, int>>

    // mark the path with colons
    for (const auto &pos : solution)
    {
        map[pos.first][pos.second] = ':';
    }

    // print the grid
    print_grid(map);

    return 0;
}
