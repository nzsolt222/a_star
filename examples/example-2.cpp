#include "astar.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <ctime>

/**
 * @brief A 3D node.
 */
struct Node : public astar::Neighbour<Node *>
{
    Node(int x, int y, int z, bool accesable = true)
        : x_(x), y_(y), z_(z), accessable_(accesable)
    {
    }

    int x_;
    int y_;
    int z_;
    bool accessable_;
};

using Grid = std::vector<std::vector<std::vector<Node>>>;

bool operator==(const Node &lhs, const Node &rhs)
{
    return lhs.x_ == rhs.x_ && lhs.y_ == rhs.y_ && lhs.z_ == rhs.z_;
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

        auto heur = manhattan(neighbour->x_, goal->x_, neighbour->y_, goal->y_,
                              +neighbour->z_, goal->z_);

        return Logic(Heuristic(heur), MoveCost(1),
                     Accesable(neighbour->accessable_));
    }
};

/**
 * @brief Create a 3D grid, and set the nodes' neighbours.
 *
 * @param width width of the grid
 * @param height height of the grid
 * @param length length of the grid
 *
 * @return 3D grid
 */
Grid create_map(int width, int height, int length)
{
    Grid nodes;

    // create a 3d grid
    nodes.reserve(width);
    for (int x = 0; x < width; ++x)
    {
        nodes.emplace_back();
        nodes[x].reserve(height);
        for (int y = 0; y < height; ++y)
        {
            nodes[x].emplace_back();
            nodes[x][y].reserve(length);
            for (int z = 0; z < length; ++z)
            {
                auto accesable = static_cast<bool>(rand() % 20);
                nodes[x][y].emplace_back(x, y, z, accesable);
                if (!accesable)
                {
                    std::cout << "(" << x << ", " << y << ", " << z
                              << ") node is not accesable." << std::endl;
                }
            }
        }
    }

    // direction offsets (x, y, z, ...)
    std::vector<int> offset = {
        +0, +0, +1, +0, +0, -1,

        +0, +1, +1, +0, +1, -1,

        +1, +0, +1, +1, +0, -1,

        +1, +1, +1, +1, +1, -1,

        +1, -1, +1, +1, -1, -1,

        -1, +1, +1, -1, +1, -1,

        -1, +0, +1, -1, +0, -1,

        -1, -1, +1, -1, -1, -1,

        +0, -1, +1, +0, -1, -1,

        -1, -1, +0, -1, +0, +0, -1, +1, +0,

        +1, -1, +0, +1, +0, +0, +1, +1, +0,
    };

    // check the (x,y,z) coordinate
    auto is_valid = [&](int x, int y, int z) NOEXCEPT
    {
        return x >= 0 && y >= 0 && z >= 0 && x < width && y < height &&
               z < length;
    };

    // set the neghbours
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int z = 0; z < length; ++z)
            {
                for (unsigned dir = 0; dir < offset.size(); dir += 2)
                {
                    auto nx = x + offset[dir];
                    auto ny = y + offset[dir + 1];
                    auto nz = y + offset[dir + 2];
                    if (is_valid(nx, ny, nz))
                    {
                        nodes[x][y][z].add_neighbour(&nodes[nx][ny][nz]);
                    }
                }
            }
        }
    }
    return nodes;
}

int main(int, char const *[])
{
    // set the seed value
    srand(time(NULL));

    // create a 3D grid
    std::cout << "Crate a 10x10x10 3D grid" << std::endl;
    Grid map = create_map(10, 10, 10);

    // find the path
    auto solution = astar::find(&map[0][0][0], &map[9][9][9], logic());

    // print the solution
    std::cout << "Path found: " << (solution.size() == 0 ? "False" : "True")
              << std::endl;

    for (auto &path_element : solution)
    {
        std::cout << "(" << path_element->x_ << ", " << path_element->y_ << ", "
                  << path_element->z_ << ") ";
    }
    std::cout << std::endl;

    return 0;
}
