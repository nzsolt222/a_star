#include "astar.hpp"

#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>
#include <functional>
#include <chrono>
#include <iomanip>

static const float WINDOW_WIDTH = 600;
static const float WINDOW_HEIGHT = 600;

static const float GRID_ROW = 50;
static const float GRID_COLUMN = 50;

static const float GRID_WIDTH = WINDOW_WIDTH / GRID_ROW;
static const float GRID_HEIGHT = WINDOW_HEIGHT / GRID_COLUMN;

class Node;
std::vector<std::vector<Node>> map;
std::vector<Node *> path;

enum class NodeStatus
{
    NORMAL = 0,
    START,
    GOAL,
    NOTACCESABLE,
};

NodeStatus click_status;
Node *start_node;
Node *goal_node;

class Node : public astar::Neighbour<Node *>
{
public:
    Node(int x, int y)
        : x_(x * GRID_WIDTH), y_(y * GRID_HEIGHT), status_(NodeStatus::NORMAL)
    {
    }

    bool is_accesable() const NOEXCEPT
    {
        return status_ != NodeStatus::NOTACCESABLE;
    }

    float get_x() const NOEXCEPT { return x_; }

    float get_y() const NOEXCEPT { return y_; }

    NodeStatus get_status() const NOEXCEPT { return status_; }

    void set_status(const NodeStatus &status) NOEXCEPT { status_ = status; }

private:
    float x_;
    float y_;
    NodeStatus status_;
};

bool operator==(const Node &lhs, const Node &rhs)
{
    return lhs.get_x() == rhs.get_x() && lhs.get_y() == rhs.get_y();
}

void draw_quad(float x, float y, float width, float height)
{
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();
}

/**
 * @brief Convert mouse position to grid index.
 *
 * @param x
 * @param y
 */
void get_map_index(int *x, int *y)
{
    double mouse_x, mouse_y;
    glfwGetCursorPos(glfwGetCurrentContext(), &mouse_x, &mouse_y);

    *x = mouse_x / GRID_WIDTH;
    *y = mouse_y / GRID_HEIGHT;

    if (*x >= GRID_ROW)
    {
        *x = GRID_ROW - 1;
    }

    if (*x < 0)
    {
        *x = 0;
    }

    if (*y >= GRID_COLUMN)
    {
        *y = GRID_COLUMN - 1;
    }

    if (*y < 0)
    {
        *y = 0;
    }
}

struct logic
{
    inline astar::Logic operator()(const Node *node, const Node *neighbour,
                                   const Node *goal) const NOEXCEPT
    {
        using namespace astar;

        auto heur = astar::manhattan(
            neighbour->get_x() / GRID_WIDTH, neighbour->get_y() / GRID_HEIGHT,
            goal->get_x() / GRID_WIDTH, goal->get_y() / GRID_HEIGHT);

        auto line =
            (std::abs(neighbour->get_x() - node->get_x()) / GRID_WIDTH) +
            (std::abs(neighbour->get_y() - node->get_y()) / GRID_HEIGHT);

        return Logic(Heuristic(heur),
                     line == 2 ? MoveCost(1.4142) : MoveCost(1),
                     Accesable(neighbour->is_accesable()));
    }
};

void update()
{
    bool pressed =
        glfwGetMouseButton(glfwGetCurrentContext(), GLFW_MOUSE_BUTTON_1);

    static int last_x;
    static int last_y;
    int now_x, now_y;
    get_map_index(&now_x, &now_y);

    if (pressed)
    {
        switch (click_status)
        {
            case NodeStatus::GOAL:
                if (map[now_x][now_y].get_status() == NodeStatus::NORMAL ||
                    map[now_x][now_y].get_status() == NodeStatus::GOAL)
                {
                    map[last_x][last_y].set_status(NodeStatus::NORMAL);
                    map[now_x][now_y].set_status(click_status);
                    goal_node = &map[now_x][now_y];
                    get_map_index(&last_x, &last_y);
                }
                break;
            case NodeStatus::START:
                if (map[now_x][now_y].get_status() == NodeStatus::NORMAL ||
                    map[now_x][now_y].get_status() == NodeStatus::START)
                {
                    map[last_x][last_y].set_status(NodeStatus::NORMAL);
                    map[now_x][now_y].set_status(click_status);
                    start_node = &map[now_x][now_y];
                    get_map_index(&last_x, &last_y);
                }
                break;
            case NodeStatus::NORMAL:
                if (map[now_x][now_y].get_status() != NodeStatus::START &&
                    map[now_x][now_y].get_status() != NodeStatus::GOAL)
                {
                    map[now_x][now_y].set_status(NodeStatus::NOTACCESABLE);
                    get_map_index(&last_x, &last_y);
                }
                break;
            default:
                if (map[now_x][now_y].get_status() != NodeStatus::START &&
                    map[now_x][now_y].get_status() != NodeStatus::GOAL)
                {
                    map[now_x][now_y].set_status(NodeStatus::NORMAL);
                    get_map_index(&last_x, &last_y);
                }
                break;
        }
    }
    else
    {
        get_map_index(&last_x, &last_y);
    }

    auto begin = std::chrono::high_resolution_clock::now();
    path = astar::find(start_node, goal_node, logic());
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << std::setw(8)
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     end - begin).count() << " microsecond"
              << " move: " << path.size() << std::endl;
}

void draw()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glPushMatrix();
    glTranslatef(-1, -1, 0);

    for (const auto &row : map)
    {
        for (const auto &node : row)
        {
            switch (node.get_status())
            {
                case NodeStatus::START:
                    glColor3f(0.1f, 0.1f, 0.8f);
                    draw_quad(node.get_x(), node.get_y(), GRID_WIDTH,
                              GRID_HEIGHT);
                    break;
                case NodeStatus::GOAL:
                    glColor3f(1.f, 0.5f, 0.f);
                    draw_quad(node.get_x(), node.get_y(), GRID_WIDTH,
                              GRID_HEIGHT);
                    break;
                case NodeStatus::NOTACCESABLE:
                    glColor3f(0.8f, 0.8f, 0.8f);
                    draw_quad(node.get_x(), node.get_y(), GRID_WIDTH,
                              GRID_HEIGHT);
                    break;
                case NodeStatus::NORMAL:
                    // draw nothing
                    break;
            }
        }
    }

    glColor3f(0.6f, 0.6f, 0.6f);

    glBegin(GL_LINES);
    for (int i = 0; i < GRID_COLUMN; ++i)
    {
        glVertex2f(0, i * GRID_HEIGHT);
        glVertex2f(WINDOW_WIDTH, i * GRID_HEIGHT);
    }
    glEnd();

    glBegin(GL_LINES);
    for (int i = 0; i < GRID_ROW; ++i)
    {
        glVertex2f(i * GRID_WIDTH, 0);
        glVertex2f(i * GRID_WIDTH, WINDOW_HEIGHT);
    }
    glEnd();

    if (path.size() != 0)
    {
        glColor3f(0.2f, 1.f, 0.5f);
        glLineWidth(2);
        glBegin(GL_LINES);
        for (unsigned i = 0; i < path.size() - 1; ++i)
        {
            glVertex2f(path[i]->get_x() + GRID_WIDTH / 2,
                       path[i]->get_y() + GRID_HEIGHT / 2);
            glVertex2f(path[i + 1]->get_x() + GRID_WIDTH / 2,
                       path[i + 1]->get_y() + GRID_HEIGHT / 2);
        }
        glEnd();
        glLineWidth(1);
    }

    glPopMatrix();
}

void initGLFW()
{
    glfwSetErrorCallback([](int error, const char *description)
                         {
                             std::cerr << "[ERROR] "
                                       << "code: " << error << ' '
                                       << "description: " << description
                                       << std::endl;
                         });

    if (!glfwInit())
    {
        std::cerr << "GLFW failed to initialize!" << std::endl;
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_RESIZABLE, 0);
    GLFWwindow *window = glfwCreateWindow(
        WINDOW_WIDTH, WINDOW_HEIGHT, "pathfinding example 4", nullptr, nullptr);
    if (window == nullptr)
    {
        std::cout << "Window failed to create!" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    glfwSetFramebufferSizeCallback(window,
                                   [](GLFWwindow *, int width, int height)
                                   {
        glViewport(0, 0, width, height);
    });

    glfwSetMouseButtonCallback(
        window, [](GLFWwindow *, int button, int action, int /*mods*/)
        {
            if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS)
            {
                int x, y;
                get_map_index(&x, &y);
                click_status = map[x][y].get_status();
            }
        });

    glfwSetKeyCallback(window,
                       [](GLFWwindow *window, int key, int, int action, int)
                       {
        // ESCAPE exit
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window, true);
        }
    });
}

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
void init()
{
    initGLFW();

    // init OpenGL stuff
    glClearColor(1, 1, 1, 1);
    glOrtho(0.0f, WINDOW_WIDTH, WINDOW_HEIGHT, 0.0f, 0.0f, 1.0f);

    map = create_map(GRID_ROW, GRID_COLUMN);
    // map = build_2d_map<Node, Straight_2D>(GRID_ROW, GRID_COLUMN);

    start_node = &map[0][0];
    start_node->set_status(NodeStatus::START);

    goal_node = &map[GRID_ROW - 1][GRID_COLUMN - 1];
    goal_node->set_status(NodeStatus::GOAL);
}

int main(int, char const *[])
{
    init();
    GLFWwindow *window = glfwGetCurrentContext();

    while (!glfwWindowShouldClose(window))
    {
        update();
        draw();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
