/**
 * @file
 * @author  Nagy Zsolt nzsolt222@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Nagy Zsolt
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * This  file contains all the necessary classes for fast and efficient A*
 *pathfinding algorithm.
 */

#ifndef NZS_PATHFINDING_ASTAR_HPP
#define NZS_PATHFINDING_ASTAR_HPP

#include <algorithm>
#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <utility>
#include <functional>

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif


/**
 * @brief An implementation in C++11 of the A* algorithm.
 */
namespace astar
{
/**
 * @brief Return the manhattan distance between two points in two dimensional
 *space.
 *
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 *
 * @return the manhattan distance
 */
inline double manhattan(double x1, double y1, double x2, double y2)
{
    return std::fabs(x1 - x2) + std::fabs(y1 - y2);  // manhattan function
}

/**
 * @brief Return the manhattan distance between two points in three  dimensional
 *space.
 *
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param z1 z coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 * @param z2 z coordinate of the second point
 *
 * @return the manhattan distance
 */
inline double manhattan(double x1, double y1, double z1, double x2, double y2,
                        double z2)
{
    return std::fabs(x1 - x2) + std::fabs(y1 - y2) + std::fabs(z1 - z2);
}

/**
 * @brief Return the euclidean distance between two points in two dimensional
 *space.
 *
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 *
 * @return the euclidean distance
 */
inline double euclidean(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

/**
 * @brief Return the euclidean distance between two points in three dimensional
 *space.
 *
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param z1 z coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 * @param z2 z coordinate of the second point
 *
 * @return the euclidean distance
 */
inline double euclidean(double x1, double y1, double z1, double x2, double y2,
                        double z2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) +
                     std::pow(z1 - z2, 2));
}

/**
 * @brief Return the chebyshev distance between two points in two dimensional
 *space.
 *
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 *
 * @return the chebyshev distance
 */
inline double chebyshev(double x1, double y1, double x2, double y2)
{
    return std::max(x1 - x2, y1 - y2);
}

/**
 * @brief Return the chebyshev distance between two points in three dimensional
 *space.
 *
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param z1 z coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 * @param z2 z coordinate of the second point
 *
 * @return the chebyshev distance
 */
inline double chebyshev(double x1, double y1, double z1, double x2, double y2,
                        double z2)
{
    return std::max(std::max(x1 - x2, y1 - y2), z1 - z2);
}

/**
 * @brief A generic type safe wrapper around the Value.
 *
 * It is help for type safety. The explicit constructor prevents you
 * accidentally using a Value as a Score, but the conversion operator allows you
 * to use a Score anywhere you need a Value.
 *
 * @code
 * class Left : public Score<double>;
 * class Right : public Score<double>;
 * void example(double left, double right); // not type safe
 * void example(Left left, Right right); // type safe
 * @endcode
 *
 * @tparam Value any type
 */
template <typename Value>
class Score
{
public:
    explicit inline Score() = default;

    explicit inline Score(Value value) NOEXCEPT : value_(std::move(value)) {}

    inline operator Value() const NOEXCEPT { return value_; }

private:
    Value value_;
};

/**
 * @brief Represents the shortest distance from start node to finish node.
 * It often called G value.
 */
class MoveCost : public Score<double>
{
public:
	MoveCost(double value) : Score(value){};
};

/**
 * @brief Represents the estimated distance from actual node to finish node.
 * It often called H value.
 */
class Heuristic : public Score<double>
{
public:
	Heuristic(double value) : Score(value){};
};

/**
 * @brief Represents the accessibility from actual node to finish node.
 */
class Accesable : public Score<bool>
{
public:
	Accesable(bool value) : Score(value){};
};

/**
* @brief A summary class of @ref Heuristic, @ref MoveCost, and @ref Accesable.
* This way the user need to specify all arguments in one place and in correct
* order(@ref Score type safety).
*/
class Logic
{
public:
    Logic(Heuristic heuristic, MoveCost move_cost,
          Accesable accesable = Accesable(true)) NOEXCEPT
        : heurictic_(heuristic),
          move_cost_(move_cost),
          accesable_(accesable)
    {
    }

    inline Accesable is_accesable() const NOEXCEPT { return accesable_; }

    inline Heuristic get_heuristic() const NOEXCEPT { return heurictic_; }

    inline MoveCost get_move_cost() const NOEXCEPT { return move_cost_; }

    inline void set_accesable(Accesable accesable) NOEXCEPT
    {
        accesable_ = accesable;
    }

    inline void set_heuristic(Heuristic heuristic) NOEXCEPT
    {
        heurictic_ = heuristic;
    }

    inline void set_move_cost(MoveCost move_cost) NOEXCEPT
    {
        move_cost_ = move_cost;
    }

private:
    Heuristic heurictic_;
    MoveCost move_cost_;
    Accesable accesable_;
};

/**
 * @brief A helper class, where the user can specify the neigbours of
 * the node.
 *
 * The user can inherit this class and after he can add the neigbours using @ref
 * Neighbour::add_neighbour function.
 *
 * @tparam Iterator the type of the neighbours
 * @tparam ContainerType container type
 */
template <class Iterator, class ContainerType = std::vector<Iterator>>
class Neighbour
{
public:
    inline Neighbour(int size = 0) { neighbour_.reserve(size); }

    inline typename ContainerType::iterator begin_neighbour() NOEXCEPT
    {
        return std::begin(neighbour_);
    }

    inline typename ContainerType::iterator end_neighbour() NOEXCEPT
    {
        return std::end(neighbour_);
    }

    inline typename ContainerType::const_iterator begin_neighbour() const
        NOEXCEPT
    {
        return neighbour_.cbegin();
    }

    inline typename ContainerType::const_iterator end_neighbour() const NOEXCEPT
    {
        return neighbour_.cend();
    }

    inline void add_neighbour(Iterator it) { neighbour_.push_back(it); }

protected:
    ContainerType neighbour_;
};

/**
 * @brief It contains the necessary classes for fast and generic astar
 * algorithm.
 */
namespace details
{
template <class UserNode>
class AstarNode
{
public:
    inline AstarNode(UserNode user_node, AstarNode<UserNode> *parent_node,
		double f_value, double g_value) NOEXCEPT
        : user_node_(user_node),
          parent_node_(parent_node),
          f_value_(f_value),
          g_value_(g_value)
    {
    }

    inline AstarNode(UserNode user_node, std::size_t f_value,
                     std::size_t g_value) NOEXCEPT
        : AstarNode(user_node, nullptr, f_value, g_value)
    {
    }

    inline explicit AstarNode(UserNode user_node) NOEXCEPT
        : AstarNode(user_node, nullptr, 0, 0)
    {
    }

    inline void set_f(double f_value) NOEXCEPT { f_value_ = f_value; }

	inline void set_g(double g_value) NOEXCEPT{ g_value_ = g_value; }

    inline void set_parent(AstarNode<UserNode> *parent) NOEXCEPT
    {
        parent_node_ = parent;
    }

	inline double get_f() const NOEXCEPT{ return f_value_; }

	inline double get_g() const NOEXCEPT{ return g_value_; }

    inline UserNode get_user_node() const NOEXCEPT { return user_node_; }

    inline AstarNode<UserNode> *get_parent() NOEXCEPT { return parent_node_; }

    inline const AstarNode<UserNode> *get_parent() const NOEXCEPT
    {
        return parent_node_;
    }

    template <typename MemoryCont, typename... Args>
    inline static AstarNode<UserNode> *make_ptr(MemoryCont &cont, UserNode node,
                                                Args &&... args)
    {
        auto ptr = new AstarNode<UserNode>(node, std::forward<Args>(args)...);
        cont.emplace_back(ptr);
        return ptr;
    }

private:
    UserNode user_node_;
    AstarNode<UserNode> *parent_node_;
	double f_value_;  // G + H
	double g_value_;  // move cost
};

template <typename Node>
class OpenList
{
public:
    using value_type = Node;

    inline void add(value_type value)
    {
        container_.push_back(std::move(value));
        std::push_heap(std::begin(container_), std::end(container_),
                       comparator_);
    }

    inline value_type remove()
    {
        std::pop_heap(std::begin(container_), std::end(container_),
                      comparator_);

        auto ret = container_.back();
        container_.pop_back();
        return ret;
    }

    inline void repair_heap()
    {
        std::make_heap(std::begin(container_), std::end(container_),
                       comparator_);
    }

    inline bool empty() const NOEXCEPT { return container_.empty(); }

    inline void clear() NOEXCEPT { container_.clear(); }

    template <typename NodeType>
    inline typename std::vector<value_type>::iterator find_by_user_node(
        NodeType node)
    {
        return std::find_if(std::begin(container_), std::end(container_),
                            [&](const value_type &lhs)
                            {
            return lhs->get_user_node() == node;
        });
    }

    inline typename std::vector<value_type>::const_iterator begin() const
        NOEXCEPT
    {
        return std::begin(container_);
    }

    inline typename std::vector<value_type>::const_iterator end() const NOEXCEPT
    {
        return std::end(container_);
    }

private:
    std::vector<value_type> container_;

    struct Comparator
    {
        inline bool operator()(const value_type &lhs,
                               const value_type &rhs) const NOEXCEPT
        {
            // return !(lhs->get_f() < rhs->get_f());
            return lhs->get_f() >= rhs->get_f();
        }
    } comparator_;
};

template <typename Key>
struct pathfinding_hash
{
    inline std::size_t operator()(const Key &key) const NOEXCEPT
    {
        static const std::size_t shift =
            (std::size_t)std::log2(1 + sizeof(Key));
        return (std::size_t)(key) >> shift;
    }
};

template <typename Key, typename DataType>
class ClosedListUnorderedMap
{
public:
    using value_type = DataType;
    using key_type = Key;
    using container_type =
        std::unordered_map<Key, DataType, pathfinding_hash<Key>>;

    inline void add(value_type value)
    {
        container_.insert(std::make_pair(value->get_user_node(), value));
    }

    inline void clear() NOEXCEPT { container_.clear(); }

    inline typename container_type::iterator find_by_user_node(key_type key)
    {
        return container_.find(key);
    }

    inline typename container_type::const_iterator begin() const NOEXCEPT
    {
        return std::begin(container_);
    }

    inline typename container_type::const_iterator end() const NOEXCEPT
    {
        return std::end(container_);
    }

private:
    container_type container_;
};

template <typename UserNode>
inline auto reconstruct_path(UserNode *goal)
    -> std::vector<decltype(goal -> get_user_node())>
{
    std::vector<decltype(goal->get_user_node())> solution;
    solution.push_back(goal->get_user_node());

    auto parents = goal->get_parent();
    while (parents != nullptr)
    {
        solution.push_back(parents->get_user_node());
        parents = parents->get_parent();
    }

    std::reverse(std::begin(solution), std::end(solution));

    return solution;
}

struct GridNode : Neighbour<GridNode *>
{
    GridNode(int x, int y) NOEXCEPT : x(x), y(y) {}
    int x;
    int y;
};

inline bool operator==(const GridNode &lhs, const GridNode &rhs) NOEXCEPT
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

}  // details

/**
 * @brief Find the path from start node to goal node.
 *
 * @tparam UserNode
 * @tparam Func a function what return with @ref Logic
 * @param start start node
 * @param goal finish node
 * @param logic_fun a function what return with @ref Logic
 *
 * @return the path
 */
template <typename UserNode, typename Func>
std::vector<UserNode *> find(UserNode *start, UserNode *goal,
                             const Func &logic_fun)
{
    // if start or goal node is not specified
    if (start == nullptr || goal == nullptr)
    {
        // return with empty path
        return {};
    }

    // a few alias
    using Node = details::AstarNode<UserNode *>;
    using Node_ptr = Node *;
    using Node_uptr = std::unique_ptr<Node>;
    using OpenList = details::OpenList<Node_ptr>;
    using ClosedList = details::ClosedListUnorderedMap<UserNode *, Node_ptr>;

    OpenList open_list_;
    ClosedList closed_list_;
    std::vector<Node_uptr> all_node;  // just for memory manager

    // add start node to the open list
    open_list_.add(Node::make_ptr(all_node, start));

    // while open_list_ is not empty
    while (!open_list_.empty())
    {
        // find the lowest f_value in open_list_ and remove it.
        auto lowest_node_ptr = open_list_.remove();

        // if we reached the target
        if (*lowest_node_ptr->get_user_node() == *goal)
        {
            // make the path
            return reconstruct_path(lowest_node_ptr);
        }

        // add the current node to the closed list
        closed_list_.add(lowest_node_ptr);

        // iterate over all neighbours
        auto end = lowest_node_ptr->get_user_node()->end_neighbour();
        for (auto it = lowest_node_ptr->get_user_node()->begin_neighbour();
             it != end; ++it)
        {
            // get the actual neigbour
            auto user_neighbour_ptr = *it;

            // call the logic funcion
            Logic logic = logic_fun(lowest_node_ptr->get_user_node(),
                                    user_neighbour_ptr, goal);

            // if the actual neghbour is not accessible
            if (!logic.is_accesable())
            {
                // skip this node
                continue;
            }

            // if the current neighbour is in the closed list
            if (closed_list_.find_by_user_node(user_neighbour_ptr) !=
                closed_list_.end())
            {
                // then skip it, because that node already explored
                continue;
            }

            // calculate the current neighbour possible g_value
            double tentative_g_value =
                lowest_node_ptr->get_g() + logic.get_move_cost();

            // find the current neighbour in open list
            auto in_open = open_list_.find_by_user_node(user_neighbour_ptr);

            // if it is not in the open list
            if (in_open == open_list_.end())
            {
                // create a new AstarNode
                auto current_neighbour_ptr = Node::make_ptr(
                    all_node, user_neighbour_ptr, lowest_node_ptr,
                    tentative_g_value + logic.get_heuristic(),
                    tentative_g_value);

                // add it to the open list
                open_list_.add(current_neighbour_ptr);
            }
            else if (tentative_g_value < (*in_open)->get_g())
            {
                // set the new values of the node
                (*in_open)->set_parent(lowest_node_ptr);
                (*in_open)->set_f(tentative_g_value + logic.get_heuristic());
                (*in_open)->set_g(tentative_g_value);

                // fix the heap, because we modified the f value.
                open_list_.repair_heap();
            }
        }
    }

    // the path not found, return an empty path
    return {};
}

/**
 * @brief A general callable struct wich return the offset position of the
 * neigbours.
 */
struct Direction
{
    virtual std::vector<int> operator()() const NOEXCEPT = 0;
};

/**
 * @brief Return the diagonal offset positions.
 */
struct Diagonal : public Direction
{
    inline std::vector<int> operator()() const NOEXCEPT override
    {
        return {
            +1, -1, +1, +1, -1, +1, -1, -1,
        };
    };
};

/**
 * @brief Return the straight offset positions.
 */
struct Straight : public Direction

{
    inline std::vector<int> operator()() const NOEXCEPT override
    {
        return {
            +0, -1, +1, +0, +0, +1, -1, +0,
        };
    };
};

/**
 * @brief Return all the eight offset positions.
 */
typedef struct DiagonalAndStraight : public Direction

{
    inline std::vector<int> operator()() const NOEXCEPT override
    {
        return {
            +0, -1, +1, -1, +1, +0, +1, +1, +0, +1, -1, +1, -1, +0, -1, -1,
        };
    };
} AllDirection;

/**
 * @brief A helper class of 2D grid type pathfinding.
 *
 * The default heuristic is the manhattan function and the default move cost is
 * 1 and every node is accesable.
 */
class Grid
{
public:
    using LogicFunc = std::function<double(int, int, int, int)>;

    /**
     * @brief Create a grid with the specified width and height and set the
     * neighbours.
     *
     * @param width the width of the grid
     * @param height the height of the grid
     * @param direction the offsets of the neighbours
     */
    Grid(int width, int height, const Direction &direction = Straight())
        : width_(width), height_(height), logic_(this)
    {
        nodes_.reserve(width_);
        for (int x = 0; x < width_; ++x)
        {
            nodes_.emplace_back();
            nodes_[x].reserve(height);
            for (int y = 0; y < height_; ++y)
            {
                nodes_[x].emplace_back(x, y);
            }
        }

        make_neigbour(direction);

        heuristic_ = [](int x1, int y1, int x2, int y2)
        {
            return manhattan(x1, y1, x2, y2);
        };
        move_cost_ = [](int, int, int, int)
        {
            return 1;
        };
        accesable_ = [](int, int, int, int)
        {
            return true;
        };
    }

    void set_accesable(LogicFunc func) { accesable_ = std::move(func); }
    void set_move_cost(LogicFunc func) { move_cost_ = std::move(func); }
    void set_heuristic(LogicFunc func) { heuristic_ = std::move(func); }

    /**
     * @brief Find the path from start node to goal node.
     *
     * @param x1 x coordinate of the first point
     * @param y1 y coordinate of the first point
     * @param x2 x coordinate of the second point
     * @param y2 y coordinate of the second point
     *
     * @return the path
     */
    std::vector<std::pair<int, int>> find(int x1, int y1, int x2, int y2)
    {
        auto start = &(nodes_.at(x1).at(y1));
        auto goal = &(nodes_.at(x2).at(y2));

        auto path = astar::find(start, goal, logic_);
        std::vector<std::pair<int, int>> sol;
        sol.reserve(path.size() / 2);
        for (const auto &elem : path)
        {
            sol.emplace_back(elem->x, elem->y);
        }
        return sol;
    }

private:
    using GridNode = details::GridNode;
    int width_;
    int height_;
    std::vector<std::vector<GridNode>> nodes_;
    LogicFunc heuristic_;
    LogicFunc move_cost_;
    LogicFunc accesable_;

    void make_neigbour(const Direction &direction)
    {
        auto is_valid = [this](int x, int y) NOEXCEPT
        {
            return x >= 0 && y >= 0 && x < width_ && y < height_;
        };

        auto directions = direction();

        for (unsigned x = 0; x < nodes_.size(); ++x)
        {
            for (unsigned y = 0; y < nodes_[x].size(); ++y)
            {
                for (unsigned dir = 0; dir < directions.size(); dir += 2)
                {
                    if (is_valid(x + directions[dir], y + directions[dir + 1]))
                    {
                        nodes_[x][y].add_neighbour(
                            &*(nodes_[x + directions[dir]].begin() + y +
                               directions[dir + 1]));
                    }
                }
            }
        }
    }

    struct logic
    {
        logic(Grid *grid) : grid_(grid) {}

        inline Logic operator()(GridNode *node, GridNode *neighbour,
                                GridNode *goal) const NOEXCEPT
        {
            return Logic(Heuristic(grid_->heuristic_(neighbour->x, neighbour->y,
                                                     goal->x, goal->y)),
                         MoveCost(grid_->move_cost_(
                             node->x, node->y, neighbour->x, neighbour->y)),
                         Accesable(grid_->accesable_(
                             node->x, node->y, neighbour->x, neighbour->y)));
        }

    private:
        Grid *grid_;
    };

    logic logic_;
};

}  // astar

#endif  // NZS_PATHFINDING_ASTAR_HPP
