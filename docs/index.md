# A* Pathfinding Algorithm

## Introduction
This project involves implementing the A* pathfinding algorithm in C++ using a grid-based environment. The goal of the project is to understand how the A* algorithm works in practice and to apply it using clean, modular, and modern C++ code.

The program represents a two-dimensional grid where each cell can either be free or blocked by an obstacle. Given a start position and a goal position, the algorithm searches for the shortest valid path while avoiding obstacles and staying within the grid boundaries. If a path cannot be found, the program will indicate that no valid route exists.

## Week 1 – Grid Representation and Core Data Structures

In Week 1, I focused on establishing the data structures and constraints required for the A* pathfinding algorithm before implementing the algorithm itself. The main goal was to create a grid representation that supports safe neighbour expansion, obstacle handling, and boundary validation, all of which are essential for A* to function correctly.

### Grid Representation

I implemented the environment as a two-dimensional grid using a `std::vector<std::vector<int>>`. Each cell in the grid represents a node in the search space, where a value of `0` indicates a traversable cell and a value of `1` represents an obstacle. This structure allows direct access to neighbouring cells using row and column indices, which aligns naturally with grid-based pathfinding.

Using a dynamic 2D vector allows the grid size to be configured at runtime and avoids manual memory management, while still providing efficient indexed access required by the algorithm.

### Position Abstraction

To represent nodes within the grid, I created a `Position` structure containing row and column coordinates. This abstraction is used consistently throughout the project to describe the start node, goal node, neighbouring nodes, and the final path returned by the algorithm. Keeping this structure simple ensures that positions can be passed and compared efficiently during the search process.

### Boundary Validation

A key requirement of A* is safe neighbour expansion. When exploring adjacent nodes, it is possible to generate positions that lie outside the grid. To handle this, I implemented an `inBounds` function that verifies whether a given position lies within the valid grid limits.

This function acts as a guard during neighbour generation and prevents invalid memory access when evaluating candidate nodes. Centralising this check within the `Grid` class ensures that all future pathfinding logic can rely on consistent boundary validation.

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

#### 5) Initial test in `main.cpp`
![Main test](images/main_test.png)

- I wrote a small test to confirm that obstacles and bounds checks behave correctly.
- I have not implemented the A* algorithm yet — Week 1 was only about setting up the foundation.

---

### Outcome

By the end of Week 1, I had a working grid representation, a clean project structure, and core helper functions that will make the A* implementation safer and easier in Week 2.
