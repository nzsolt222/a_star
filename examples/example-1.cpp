#include "astar.hpp"

#include <iostream>
#include <vector>
#include <ctime>

/**
 * @brief A 2D node.
 */
struct Node : public astar::Neighbour<Node *>
{
    Node(int x, int y, bool accesable)
        : x_(x), y_(y), accesable_(accesable), path_(false)
    {
    }

    int x_;
    int y_;
    bool accesable_;
    bool path_;
};

using Grid = std::vector<std::vector<Node>>;

inline bool operator==(const Node &lhs, const Node &rhs) NOEXCEPT
{
    return lhs.x_ == rhs.x_ && lhs.y_ == rhs.y_;
}

/**
 * @brief The logic of the A* algorithm.
 */
struct logic
{
    inline astar::Logic operator()(const Node * /*node*/, const Node *neighbour,
                                   const Node *goal) const NOEXCEPT
    {
        using namespace astar;

        return Logic(Heuristic(manhattan(neighbour->x_, neighbour->y_, goal->x_,
                                         goal->y_)),
                     MoveCost(1), Accesable(neighbour->accesable_));
    }
};

/**
 * @brief Create a 2D grid, and set the nodes' neighbours.
 *
 * @param width width of the grid
 * @param height height of the grid
 *
 * @return 2D grid
 */
Grid create_map(int width, int height)
{
    Grid nodes;

    // create a 2d grid
    nodes.reserve(width);
    for (int x = 0; x < width; ++x)
    {
        nodes.emplace_back();
        nodes[x].reserve(height);
        for (int y = 0; y < height; ++y)
        {
            nodes[x].emplace_back(x, y, static_cast<bool>(rand() % 10));
        }
    }

    // straight direction offsets (x, y, ...)
    int offset[8] = {+0, -1, +1, +0, +0, +1, -1, +0};

    // check the (x,y) coordinate
    auto is_valid = [&](int x, int y) NOEXCEPT
    {
        return x >= 0 && y >= 0 && x < width && y < height;
    };

    // set the neghbours
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int dir = 0; dir < 8; dir += 2)
            {
                auto nx = x + offset[dir];
                auto ny = y + offset[dir + 1];
                if (is_valid(nx, ny))
                {
                    nodes[x][y].add_neighbour(&nodes[nx][ny]);
                }
            }
        }
    }
    return nodes;
}

/**
 * @brief Print a 2D grid with path.
 *
 * @param grid 2D grid
 */
void print_map(const Grid &grid)
{
    for (unsigned x = 0; x < grid.size(); ++x)
    {
        for (unsigned y = 0; y < grid[x].size(); ++y)
        {
            if (grid[x][y].path_)
            {
                std::cout << "O";
            }
            else if (grid[x][y].accesable_ == false)
            {
                std::cout << "#";
            }
            else
            {
                std::cout << " ";
            }
        }
        std::cout << std::endl;
    }
}

int main(int, char const *[])
{
    // set the seed value
    srand(time(NULL));

    // create a 2D grid
    const int map_size = 20;
    Grid nodes = create_map(map_size, map_size);

    // find the path
    auto solution =
        astar::find(&nodes[0][0], &nodes[map_size - 1][map_size - 1], logic());

    // mark the path
    for (auto &element : solution)
    {
        element->path_ = true;
    }

    // print the grid
    print_map(nodes);

    return 0;
}
