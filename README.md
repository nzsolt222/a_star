# A\* Pathfinding

It is an implementation in C++11 of the A-star(A\*) algorithm. This is a header only library. It can be used any type of graph based searching.

## Contents

- [Examples](#examples)
- [Tutorial](#tutorial)
    - [2D Graph based searching](#2d-graph-based-searching)
    - [2D Grid based searching](#2d-grid-based-searching)
- [Screenshot](#screenshot)
- [License](#license)

## Examples

You can find many examples under the examples folder.

| Filename         | Description                      |
|------------------|----------------------------------|
| example-0.cpp    | 2D grid  based example           |
| example-1.cpp    | 2D graph based example           |
| example-2.cpp    | 3D graph based example           |
| example-3.cpp    | 2D graph based example with gui  |

## Tutorial

### 2D Graph based searching

1\. Let's create a Node and inherit the astart::Neigbour class.
```cpp
struct Node : astar::Neighbour<Node *>
{
    int x;
    int y;
    bool accesable;
};
```
2\. Implement the equal operator.
```cpp
bool operator==(const Node &lhs, const Node &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}
```
3\. Create a logic functor.
```cpp
struct logic
{
    astar::Logic operator()(const Node *node, const Node *neighbour,
                            const Node *goal) const noexcept
    {
        using namespace astar;
        // heuristic: from node to goal
        // move cost: from node to neighbour
        // accesable: from node to neighbour
        return Logic(Heuristic(manhattan(node->x, node->y, goal->x, goal->y)),
                     MoveCost(1), Accesable(neighbour->accesable));
    }
};
```
4\. Create a graph.
```cpp
for (int x = 0; x < 100; ++x)
{
    for (int y = 0; y < 100; ++y)
    {
        graph[x][y].x = x;
        graph[x][y].y = y;
        graph[x][y].accesable = (bool)rand() % 5;
    }
}

// add neighbours
int offset[8] = {+1, -1, +1, +1, -1, +1, -1, -1};
auto is_valid = [](int x, int y)
{
    return x >= 0 && y >= 0 && x < 100 && y < 100;
};

for (int x = 0; x < 100; ++x)
{
    for (int y = 0; y < 100; ++y)
    {
        for (int dir = 0; dir < 8; dir += 2)
        {
            auto nx = x + offset[dir];
            auto ny = y + offset[dir + 1];
            if (is_valid(nx, ny))
            {
                graph[x][y].add_neighbour(&graph[nx][ny]);
            }
        }
    }
}
```
5\. Find the path.
```cpp
std::vector<Node *> solution =
    astar::find(&graph[0][0], &graph[99][99], logic());
```

### 2D Grid based searching
1\. Create an astar::Grid and your grid structure. 
```cpp
astar::Grid grid(12, 9);
char map[12][9] = {
    {'+', '-', '-', '-', '-', '-', '-', '-', '+'},  // or std::vector
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
```
2\. Set the heuristic(default manhattan), accesable(default true) and the move cost(default 1).
```cpp
grid.set_accesable([&](int x1, int y1, int, int)
                   {
                       return map[x1][y1] == ' ';
                   });
// grid.set_heuristic
// grid.set_move_cost
```
3\. Find the path.
```cpp
std::vector<std::pair<int, int>> solution = grid.find(1, 1, 10, 7);
```

## Screenshot
![Example 1](screenshot/example3.gif?raw=true)

## License  

This software is released under the MIT License, see license.txt
