# A* Pathfinding Algorithm

## Introduction
This project involves implementing the A* pathfinding algorithm in C++ using a grid-based environment. The goal of the project is to understand how the A* algorithm works in practice and to apply it using clean, modular, and modern C++ code.

The program represents a two-dimensional grid where each cell can either be free or blocked by an obstacle. Given a start position and a goal position, the algorithm searches for the shortest valid path while avoiding obstacles and staying within the grid boundaries. If a path cannot be found, the program will indicate that no valid route exists.

## Week 1 – Grid Representation and Core Data Structures

In Week 1, I focused on establishing the data structures and constraints required for the A* pathfinding algorithm before implementing the algorithm itself. The main goal was to create a grid representation that supports safe neighbour expansion, obstacle handling, and boundary validation, all of which are essential for A* to function correctly.

### Grid Representation

I implemented the environment as a two-dimensional grid using a `std::vector<std::vector<int>>`. Each cell in the grid represents a node in the search space, where a value of `0` indicates a traversable cell and a value of `1` represents an obstacle. This structure allows direct access to neighbouring cells using row and column indices, which aligns naturally with grid-based pathfinding. 

![Grid](images/Grid.h_w1.png)

Using a dynamic 2D vector allows the grid size to be configured at runtime and avoids manual memory management, while still providing efficient indexed access required by the algorithm.

### Position Abstraction

To represent nodes within the grid, I created a `Position` structure containing row and column coordinates. This abstraction is used consistently throughout the project to describe the start node, goal node, neighbouring nodes, and the final path returned by the algorithm. Keeping this structure simple ensures that positions can be passed and compared efficiently during the search process.

![Main test](images/position.h_w1.png)

### Boundary Validation

A key requirement of A* is safe neighbour expansion. When exploring adjacent nodes, it is possible to generate positions that lie outside the grid. To handle this, I implemented an `inBounds` function that verifies whether a given position lies within the valid grid limits.

This function acts as a guard during neighbour generation and prevents invalid memory access when evaluating candidate nodes. Centralising this check within the `Grid` class ensures that all future pathfinding logic can rely on consistent boundary validation.

![Main test](images/Grid.cpp_w1.png)

### Walkability and Obstacles

In addition to boundary checks, I implemented an `isWalkable` function to determine whether a cell can be traversed. This function combines boundary validation with obstacle checking, allowing the pathfinding algorithm to immediately discard blocked or invalid nodes during expansion.

Obstacles are inserted into the grid using a dedicated function, which modifies the underlying grid data while maintaining encapsulation. This setup mirrors how A* treats blocked nodes as non-expandable during the search process.

### Initial Validation

To verify the correctness of the grid representation and helper functions, I added a minimal test in `main.cpp` that checks boundary conditions and obstacle handling. Although no pathfinding logic is implemented at this stage, this validation ensures that the grid behaves correctly before introducing the A* algorithm in later weeks.

Screenshots included in this section show the project structure, grid implementation, and helper functions completed during Week 1.

---

### Week 1 Outcome

By the end of Week 1, I had a grid-based environment capable of supporting A* pathfinding, including node representation, boundary checks, and obstacle handling. This foundation ensures that the algorithm implementation in Week 2 can focus purely on search logic, cost calculation, and heuristic evaluation without revisiting structural concerns.
- `isWalkable` checks both bounds and whether a cell is blocked.


- I wrote a small test to confirm that obstacles and bounds checks behave correctly.
- I have not implemented the A* algorithm yet — Week 1 was only about setting up the foundation.

---

### Outcome

By the end of Week 1, I had a working grid representation, a clean project structure, and core helper functions that will make the A* implementation safer and easier in Week 2.

## Week 2 – Implementing the A* Algorithm (Core Search)

In Week 2, I implemented the core A* pathfinding algorithm on a grid with 4-direction movement (up, down, left, right). The main goal this week was to move from a working grid environment to a working shortest-path search that correctly returns a path from Start to Goal while avoiding obstacles.

### Neighbour Expansion (Search Space)

Before writing the full A* loop, I implemented neighbour generation as a dedicated helper function. From a given `Position`, I generate the 4 candidate neighbours and then filter them using `Grid::isWalkable(...)`. This ensures that the algorithm only expands valid nodes (inside the grid and not blocked) and prevents out-of-bounds access during the search.

This step was important because A* correctness depends on expanding exactly the valid neighbouring states in the search space.

### Heuristic Function (Manhattan Distance)

I added a heuristic function using Manhattan distance:

- `h(n) = |row_n - row_goal| + |col_n - col_goal|`

This heuristic matches the movement rules (4- directional movement, no diagonals), so it remains admissible and consistent in this environment. In practice, this means A* is guided toward the goal while still guaranteeing an optimal shortest path on an unweighted 4-direction grid.

### Open Set, Cost Tracking, and Parent Mapping

I implemented the main A* loop using these structures:

- **Open set**: a `priority_queue` ordered by the lowest `f(n) = g(n) + h(n)` so that the most promising node is expanded first.
- **`gScore`**: stores the best known cost from the start to each explored position.
- **`cameFrom`**: stores parent pointers so that once the goal is reached, the path can be reconstructed.
- **Closed set**: prevents re-expanding nodes that have already been processed, reducing duplicate work and improving efficiency.

Each step to a neighbour on the grid is treated as a uniform cost of `1`, which fits the current grid model (unweighted traversal).

### Path Reconstruction and Output

When the goal is reached, I reconstruct the final path by walking backwards through `cameFrom` from the goal to the start, then reversing the result so it is in start-to-goal order. This produces the final `std::vector<Position>` path that the rest of the program can display.
```cpp
std::vector<Position> Pathfinder::reconstructPath(
    const std::unordered_map<Position, Position, PositionHash>& cameFrom,
    Position current) const
{
    std::vector<Position> path;
    path.push_back(current);

    while (cameFrom.count(current)) {
        current = cameFrom.at(current);
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}
```

To validate the output, I added:

- A printed path length (`path.size()`)
- A visual grid overlay where:
  - `S` = start
  - `G` = goal
  - `#` = obstacle
  - `*` = returned path cells

This made it easy to confirm that the returned path is continuous, avoids obstacles, and reaches the goal.

---

### Screenshots (Week 2)

#### 1) Neighbour generation + filtering
![Neighbour generation](images/week2_neighbours.png)

#### 2) Manhattan heuristic function
![Heuristic function](images/week2_heuristic.png)

#### 3) A* main loop (`openSet`, `gScore`, `cameFrom`, `closed`)
![A* main loop](images/week2_astar_loop.png)

#### 4) Path reconstruction
![Path reconstruction](images/week2_reconstruct.png)

#### 5) Output (path overlay)
![A* output](images/week2_output.png)

---

### Week 2 Outcome

By the end of Week 2, I had a working A* implementation that can find and return a shortest path on a grid with obstacles using a Manhattan heuristic. The algorithm now produces both a coordinate path and a visual grid overlay, which will make further testing and future improvements (such as diagonals or weighted cells) easier to validate.

