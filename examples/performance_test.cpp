#include "astar.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <functional>
#include <iomanip>
#include <string>
#include <string.h>

const std::string currentDateTime();

std::string file_path = "astar_performance_test_" + currentDateTime() + ".txt";
static std::ofstream file(file_path);

/**
 * @brief Get current date/time, format is YYYY-MM-DD.HH:mm:ss.
 *
 * @return current date and time
 */
const std::string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

/**
 * @brief Print the arguments to standard output and the log file.
 */
void print()
{
    std::cout << std::flush;
    file << std::flush;
}

/**
 * @brief Print the arguments to standard output and the log file.
 *
 * @param head
 * @param tail
 */
template <typename Head, typename... Tail>
void print(Head head, Tail... tail)
{
    std::cout << head;
    file << head;
    print(tail...);
}

/**
 * @brief
 */
struct Dimension
{
    Dimension(int row = 0, int column = 0) : row_(row), column_(column) {}

    int row_;
    int column_;
};

/**
 * @brief A 2D node.
 */
struct Node : public astar::Neighbour<Node *>
{
    Node(int x, int y) : x_(x), y_(y), accesable_(true) {}

    int x_;
    int y_;
    bool accesable_;
};

bool operator==(const Node &lhs, const Node &rhs)
{
    return lhs.x_ == rhs.x_ && lhs.y_ == rhs.y_;
}

/**
 * @brief The logic of the A* algorithm.
 */
struct logic
{
    inline astar::Logic operator()(const Node *, const Node *neighbour,
                                   const Node *goal) const NOEXCEPT
    {
        using namespace astar;

        auto heur =
            astar::manhattan(neighbour->x_, neighbour->y_, goal->x_, goal->y_);

        return Logic(Heuristic(heur), MoveCost(1),
                     Accesable(neighbour->accesable_));
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
std::vector<std::vector<Node>> create_map(int width, int height)
{
    std::vector<std::vector<Node>> nodes;

    // create a 2d grid
    nodes.reserve(width);
    for (int x = 0; x < width; ++x)
    {
        nodes.emplace_back();
        nodes[x].reserve(height);
        for (int y = 0; y < height; ++y)
        {
            nodes[x].emplace_back(x, y);
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
 * @brief Run a test.
 *
 * @param row width of the grid
 * @param column height of the grid
 * @param blocks not accesable node positions
 */
void make_test(int row, int column,
               std::vector<Dimension> blocks = std::vector<Dimension>())
{
    static const int TRY = 5;
    decltype(std::chrono::high_resolution_clock::now() -
             std::chrono::high_resolution_clock::now()) duration{0};

    auto grid = create_map(row, column);

    for (const auto &dimension : blocks)
    {
        grid[dimension.row_][dimension.column_].accesable_ = false;
    }

    for (int i = 0; i < TRY; ++i)
    {
        auto begin = std::chrono::high_resolution_clock::now();
        auto solution =
            astar::find(&grid[0][0], &grid[row - 1][column - 1], logic());
        auto end = std::chrono::high_resolution_clock::now();

        duration += (end - begin);
    }

    duration /= TRY;

    auto header =
        std::to_string(row) + "x" + std::to_string(column) + " time: ";

    print(
        "\tsize: ", std::setw(16), header, std::setw(3),
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(),
        " ms (", std::setw(3),
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count(),
        " ns)", "\n");
}

int main(int, char const *[])
{
    if (file.is_open() == false)
    {
        std::cout << "Cannot open log file" << std::endl;
        // std::exit(EXIT_FAILURE);
    }

    print(
        "Performance Test!\n"
        "The start node is always the left up corner and the goal node is\n"
        "always the right down corner.\n\n");

    std::vector<Dimension> sizes = {
        {2, 2},
        {5, 5},
        {10, 10},
        {50, 50},
        {100, 50},
        {100, 100},
        {150, 150},
        {200, 200},
        {300, 300},
        {400, 400},
        {500, 500},
        {600, 600},
        {700, 700},
        {800, 800},
        {900, 900},
        {1000, 1000},
    };

    print(
        "-------------------------------goal is "
        "accesable------------------------------\n");

    for (const auto &size : sizes)
    {
        make_test(size.row_, size.column_);
    }

    print(
        "\n-----------------------------goal isn't "
        "accesable-----------------------------\n");

    const int size = sizes.size() / 2;
    for (int i = 0; i < size - 1; i++)
    {
        make_test(sizes[i].row_, sizes[i].column_,
                  {{sizes[i].row_ - 1, sizes[i].column_ - 1}});
    }

    print("\nSaved to ", file_path, "\n");

    return 0;
}
