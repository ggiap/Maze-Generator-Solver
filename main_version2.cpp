#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

// ======= Constants and Enums =======
const int SCREEN_WIDTH = 2560;
const int SCREEN_HEIGHT = 1440;
const int CELL_SIZE = 10;
const int WALL_THICKNESS = 1;

// Default maze dimensions
const int DEFAULT_MAZE_WIDTH = 201;
const int DEFAULT_MAZE_HEIGHT = 137;

// Colors
const SDL_Color WHITE = { 255, 255, 255, 255 };
const SDL_Color BLACK = { 0, 0, 0, 255 };
const SDL_Color RED = { 255, 0, 0, 255 };
const SDL_Color GREEN = { 0, 255, 0, 255 };
const SDL_Color BLUE = { 0, 0, 255, 255 };
const SDL_Color YELLOW = { 255, 255, 0, 255 };
const SDL_Color PURPLE = { 128, 0, 128, 255 };

enum class CellState {
    WALL,
    PATH,
    VISITED,
    CURRENT,
    START,
    END,
    SOLUTION
};

enum class GenerationAlgorithm {
    RECURSIVE_BACKTRACKER,
    KRUSKAL,
    PRIM,
    WILSON,
    ELLER,
    GROWING_TREE
};

enum class SolvingAlgorithm {
    DFS,
    BFS,
    A_STAR
};

// ======= Maze Structure Classes =======
struct Point {
    int x, y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    Point operator+(const Point& other) const {
        return { x + other.x, y + other.y };
    }

    std::string toString() const {
        return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
    }
};

// For using Point as a key in unordered containers
namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

struct Cell {
    CellState state = CellState::WALL;
    bool walls[4] = { true, true, true, true }; // North, East, South, West
    int distance = -1; // Used for BFS and A*
    Point parent = { -1, -1 }; // Used for path reconstruction
    float fScore = std::numeric_limits<float>::infinity(); // Used for A*
};

// Directions: North, East, South, West
const std::vector<Point> DIRECTIONS = { {0, -1}, {1, 0}, {0, 1}, {-1, 0} };

class Maze {
public:
    Maze(int width, int height)
        : width_(width), height_(height) {
        cells_.resize(height_, std::vector<Cell>(width_));
        Reset();
    }

    void Reset() {
        // Initialize all cells as walls
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                cells_[y][x].state = CellState::WALL;
                for (int i = 0; i < 4; i++) {
                    cells_[y][x].walls[i] = true;
                }
                cells_[y][x].distance = -1;
                cells_[y][x].parent = { -1, -1 };
                cells_[y][x].fScore = std::numeric_limits<float>::infinity();
            }
        }

        // Set start and end points - ensure they're odd coordinates for proper alignment
        startPoint_ = { 1, 1 };
        endPoint_ = { width_ - 2, height_ - 2 };

        // Make sure start and end points are at odd coordinates
        if (startPoint_.x % 2 == 0) startPoint_.x++;
        if (startPoint_.y % 2 == 0) startPoint_.y++;
        if (endPoint_.x % 2 == 0) endPoint_.x--;
        if (endPoint_.y % 2 == 0) endPoint_.y--;

        // Make sure they're in bounds
        startPoint_.x = std::max(1, std::min(startPoint_.x, width_ - 2));
        startPoint_.y = std::max(1, std::min(startPoint_.y, height_ - 2));
        endPoint_.x = std::max(1, std::min(endPoint_.x, width_ - 2));
        endPoint_.y = std::max(1, std::min(endPoint_.y, height_ - 2));
    }

    void CarvePathAt(int x, int y) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            cells_[y][x].state = CellState::PATH;
            // Making a cell PATH doesn't automatically remove its walls
            // That's handled separately by RemoveWall
        }
    }

    void SetCellState(int x, int y, CellState state) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            cells_[y][x].state = state;
        }
    }

    CellState GetCellState(int x, int y) const {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            return cells_[y][x].state;
        }
        return CellState::WALL;
    }

    void RemoveWall(int x, int y, int direction) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            cells_[y][x].walls[direction] = false;

            // Remove the opposite wall from the adjacent cell
            Point neighbor = { x + DIRECTIONS[direction].x, y + DIRECTIONS[direction].y };
            if (IsValidCell(neighbor.x, neighbor.y)) {
                int oppositeDir = (direction + 2) % 4; // opposite direction (N<->S, E<->W)
                cells_[neighbor.y][neighbor.x].walls[oppositeDir] = false;

                // Ensure the cell is marked as a path
                cells_[neighbor.y][neighbor.x].state = CellState::PATH;
            }
        }
    }

    bool HasWall(int x, int y, int direction) const {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            return cells_[y][x].walls[direction];
        }
        return true;
    }

    bool IsValidCell(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    const Cell& GetCell(int x, int y) const {
        return cells_[y][x];
    }

    Cell& GetCell(int x, int y) {
        return cells_[y][x];
    }

    int GetWidth() const { return width_; }
    int GetHeight() const { return height_; }

    const Point& GetStartPoint() const { return startPoint_; }
    const Point& GetEndPoint() const { return endPoint_; }

    void SetStartPoint(const Point& point) { startPoint_ = point; }
    void SetEndPoint(const Point& point) { endPoint_ = point; }

private:
    int width_;
    int height_;
    std::vector<std::vector<Cell>> cells_;
    Point startPoint_;
    Point endPoint_;
};

// ======= Maze Generator Classes =======
class MazeGenerator {
public:
    virtual ~MazeGenerator() = default;
    virtual void GenerateMaze(Maze& maze) = 0;
    virtual bool Step() = 0;
    virtual void Reset(Maze& maze) = 0;
    virtual bool IsFinished() const {}
};

class RecursiveBacktrackerGenerator : public MazeGenerator {
public:
    void GenerateMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (cellStack_.empty()) {
            return false;
        }

        Point current = cellStack_.top();
        maze_->SetCellState(current.x, current.y, CellState::CURRENT);

        // Get unvisited neighbors
        std::vector<std::pair<Point, int>> unvisitedNeighbors;
        for (int i = 0; i < 4; i++) {
            Point neighbor = { current.x + DIRECTIONS[i].x * 2, current.y + DIRECTIONS[i].y * 2 };
            if (maze_->IsValidCell(neighbor.x, neighbor.y) &&
                maze_->GetCellState(neighbor.x, neighbor.y) == CellState::WALL) {
                unvisitedNeighbors.push_back({ neighbor, i });
            }
        }

        if (!unvisitedNeighbors.empty()) {
            // Choose a random unvisited neighbor
            int randomIndex = rand() % unvisitedNeighbors.size();
            Point next = unvisitedNeighbors[randomIndex].first;
            int direction = unvisitedNeighbors[randomIndex].second;

            // Calculate wall position
            Point wallPoint = { current.x + DIRECTIONS[direction].x, current.y + DIRECTIONS[direction].y };

            // Carve the path through the wall
            maze_->CarvePathAt(wallPoint.x, wallPoint.y);

            // Mark the next cell as part of the path
            maze_->CarvePathAt(next.x, next.y);

            // Explicitly remove the walls between cells in both directions
            maze_->RemoveWall(current.x, current.y, direction);
            int oppositeDir = (direction + 2) % 4;
            maze_->RemoveWall(next.x, next.y, oppositeDir);

            // Push the next cell to stack for future exploration
            cellStack_.push(next);
        }
        else {
            // No unvisited neighbors, backtrack
            maze_->SetCellState(current.x, current.y, CellState::PATH);
            cellStack_.pop();

            if (!cellStack_.empty()) {
                Point backtrack = cellStack_.top();
                maze_->SetCellState(backtrack.x, backtrack.y, CellState::CURRENT);
            }
        }

        return !cellStack_.empty();
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;
        maze.Reset();

        // Empty the stack
        std::stack<Point> emptyStack;
        cellStack_.swap(emptyStack);

        // Start from a random point - ensure we use odd coordinates
        int startX = 1 + (rand() % ((maze.GetWidth() - 3) / 2)) * 2;
        int startY = 1 + (rand() % ((maze.GetHeight() - 3) / 2)) * 2;

        // Ensure coordinates are odd for grid alignment
        if (startX % 2 == 0) startX++;
        if (startY % 2 == 0) startY++;

        // Make sure start is in bounds
        startX = std::max(1, std::min(startX, maze.GetWidth() - 2));
        startY = std::max(1, std::min(startY, maze.GetHeight() - 2));

        Point start = { startX, startY };
        maze.CarvePathAt(start.x, start.y);
        cellStack_.push(start);
    }

    bool IsFinished() const {
        return cellStack_.empty();
    }

private:
    Maze* maze_;
    std::stack<Point> cellStack_;
};

class KruskalGenerator : public MazeGenerator {
public:
    void GenerateMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (walls_.empty()) {
            return false;
        }

        // Get the next wall
        Wall wall = walls_.back();
        walls_.pop_back();

        Point cell1 = wall.cell1;
        Point cell2 = wall.cell2;

        // Debug output
        if (debugMode_) {
            std::cout << "Considering wall between " << cell1.x << "," << cell1.y
                << " and " << cell2.x << "," << cell2.y
                << " (dir: " << wall.direction << ")" << std::endl;
        }

        // Check if the cells are in different sets
        int set1 = FindSet(cell1);
        int set2 = FindSet(cell2);

        if (set1 != set2) {
            // Calculate wall position
            Point wallPoint = {
                cell1.x + DIRECTIONS[wall.direction].x,
                cell1.y + DIRECTIONS[wall.direction].y
            };

            // Debug output
            if (debugMode_) {
                std::cout << "Removing wall at " << wallPoint.x << "," << wallPoint.y
                    << " (between sets " << set1 << " and " << set2 << ")" << std::endl;
            }

            // Carve the path at both the cells and the wall position
            maze_->CarvePathAt(cell1.x, cell1.y);
            maze_->CarvePathAt(cell2.x, cell2.y);
            maze_->CarvePathAt(wallPoint.x, wallPoint.y);

            // Remove the walls between the cells
            maze_->RemoveWall(cell1.x, cell1.y, wall.direction);
            int oppositeDir = (wall.direction + 2) % 4;
            maze_->RemoveWall(cell2.x, cell2.y, oppositeDir);

            // Merge the sets
            UnionSets(set1, set2);
        }

        return !walls_.empty();
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;
        maze.Reset();

        // Clear previous data
        walls_.clear();
        cellSets_.clear();
        setParents_.clear();

        // Initialize the maze with a grid of cells
        // Only use odd coordinates for cell positions (1,1), (1,3), (3,1), etc.
        for (int y = 1; y < maze.GetHeight(); y += 2) {
            for (int x = 1; x < maze.GetWidth(); x += 2) {
                // Mark cell as path
                maze.CarvePathAt(x, y);

                // Each cell is initially in its own set
                Point cell = { x, y };
                int setIndex = cellSets_.size();
                cellSets_[cell] = setIndex;
                setParents_[setIndex] = setIndex; // Initialize parent

                // Only add walls between valid cells (not on the boundary)
                if (x + 2 < maze.GetWidth()) {
                    // Add east wall
                    Point neighbor = { x + 2, y };
                    walls_.push_back({ cell, neighbor, 1 }); // East direction
                }

                if (y + 2 < maze.GetHeight()) {
                    // Add south wall
                    Point neighbor = { x, y + 2 };
                    walls_.push_back({ cell, neighbor, 2 }); // South direction
                }
            }
        }

        // Shuffle the walls for randomness
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(walls_.begin(), walls_.end(), g);

        if (debugMode_) {
            std::cout << "Kruskal initialized with " << walls_.size() << " walls and "
                << cellSets_.size() << " cells" << std::endl;
        }
    }

    bool IsFinished() const {
        return walls_.empty();
    }

private:
    struct Wall {
        Point cell1;
        Point cell2;
        int direction;
    };

    Maze* maze_;
    std::vector<Wall> walls_;
    std::unordered_map<Point, int> cellSets_;
    std::unordered_map<int, int> setParents_;
    bool debugMode_ = false;

    int FindSet(const Point& cell) {
        if (cellSets_.find(cell) == cellSets_.end()) {
            if (debugMode_) {
                std::cout << "Warning: Cell " << cell.x << "," << cell.y << " not found in any set" << std::endl;
            }
            return -1; // Error case
        }

        int setIndex = cellSets_[cell];
        return FindSet(setIndex);
    }

    int FindSet(int setIndex) {
        // Initialize if not found
        if (setParents_.find(setIndex) == setParents_.end()) {
            setParents_[setIndex] = setIndex;
            return setIndex;
        }

        // Path compression
        if (setParents_[setIndex] != setIndex) {
            setParents_[setIndex] = FindSet(setParents_[setIndex]);
        }

        return setParents_[setIndex];
    }

    void UnionSets(int set1, int set2) {
        int root1 = FindSet(set1);
        int root2 = FindSet(set2);

        if (root1 != root2) {
            setParents_[root2] = root1;

            if (debugMode_) {
                std::cout << "Union: set " << root2 << " now points to " << root1 << std::endl;
            }
        }
    }
};

class PrimGenerator : public MazeGenerator {
public:
    void GenerateMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (frontierCells_.empty()) {
            return false;
        }

        // Choose a random frontier cell
        int randomIndex = rand() % frontierCells_.size();
        Point current = frontierCells_[randomIndex];

        // Remove it from the frontier
        frontierCells_[randomIndex] = frontierCells_.back();
        frontierCells_.pop_back();

        // Mark the cell as part of the maze
        maze_->CarvePathAt(current.x, current.y);
        visitedCells_.insert(current);

        // Connect to a random visited neighbor
        std::vector<std::pair<Point, int>> visitedNeighbors;
        for (int i = 0; i < 4; i++) {
            Point neighbor = { current.x + DIRECTIONS[i].x * 2, current.y + DIRECTIONS[i].y * 2 };
            if (maze_->IsValidCell(neighbor.x, neighbor.y) &&
                visitedCells_.find(neighbor) != visitedCells_.end()) {
                visitedNeighbors.push_back({ neighbor, i });
            }
        }

        if (!visitedNeighbors.empty()) {
            int randNeighbor = rand() % visitedNeighbors.size();
            Point connectedCell = visitedNeighbors[randNeighbor].first;
            int direction = visitedNeighbors[randNeighbor].second;

            // Calculate wall position and remove the wall
            Point wallPoint = { current.x + DIRECTIONS[direction].x, current.y + DIRECTIONS[direction].y };
            maze_->CarvePathAt(wallPoint.x, wallPoint.y);

            // Remove the internal wall representations too
            if (direction == 0) { // North
                maze_->RemoveWall(current.x, current.y, 0); // Remove current's north wall
                maze_->RemoveWall(connectedCell.x, connectedCell.y, 2); // Remove neighbor's south wall
            }
            else if (direction == 1) { // East
                maze_->RemoveWall(current.x, current.y, 1); // Remove current's east wall
                maze_->RemoveWall(connectedCell.x, connectedCell.y, 3); // Remove neighbor's west wall
            }
            else if (direction == 2) { // South
                maze_->RemoveWall(current.x, current.y, 2); // Remove current's south wall
                maze_->RemoveWall(connectedCell.x, connectedCell.y, 0); // Remove neighbor's north wall
            }
            else if (direction == 3) { // West
                maze_->RemoveWall(current.x, current.y, 3); // Remove current's west wall
                maze_->RemoveWall(connectedCell.x, connectedCell.y, 1); // Remove neighbor's east wall
            }
        }

        // Add neighboring cells to the frontier
        for (int i = 0; i < 4; i++) {
            Point neighbor = { current.x + DIRECTIONS[i].x * 2, current.y + DIRECTIONS[i].y * 2 };
            if (maze_->IsValidCell(neighbor.x, neighbor.y) &&
                maze_->GetCellState(neighbor.x, neighbor.y) == CellState::WALL &&
                std::find(frontierCells_.begin(), frontierCells_.end(), neighbor) == frontierCells_.end() &&
                visitedCells_.find(neighbor) == visitedCells_.end()) {
                frontierCells_.push_back(neighbor);
            }
        }

        return !frontierCells_.empty();
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;
        maze.Reset();

        // Clear previous data
        frontierCells_.clear();
        visitedCells_.clear();

        // Start with a random cell
        int startX = 1 + (rand() % ((maze.GetWidth() - 2) / 2)) * 2;
        int startY = 1 + (rand() % ((maze.GetHeight() - 2) / 2)) * 2;

        Point start = { startX, startY };
        maze.CarvePathAt(start.x, start.y);
        visitedCells_.insert(start);

        // Add neighbors to frontier
        for (int i = 0; i < 4; i++) {
            Point neighbor = { start.x + DIRECTIONS[i].x * 2, start.y + DIRECTIONS[i].y * 2 };
            if (maze.IsValidCell(neighbor.x, neighbor.y)) {
                frontierCells_.push_back(neighbor);
            }
        }
    }

    bool IsFinished() const {
        return frontierCells_.empty();
    }

private:
    Maze* maze_;
    std::vector<Point> frontierCells_;
    std::unordered_set<Point> visitedCells_;
};

// ======= Maze Solver Classes =======
class MazeSolver {
public:
    virtual ~MazeSolver() = default;
    virtual void SolveMaze(Maze& maze) = 0;
    virtual bool Step() = 0;
    virtual void Reset(Maze& maze) = 0;
    virtual bool IsFinished() const = 0;
    virtual bool IsSolved() const = 0;
};

class DFSSolver : public MazeSolver {
public:
    void SolveMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (stack_.empty() || solved_) {
            return false;
        }

        Point current = stack_.top();
        stack_.pop();

        // If we've reached the end, reconstruct the path
        if (current.x == maze_->GetEndPoint().x && current.y == maze_->GetEndPoint().y) {
            solved_ = true;
            ReconstructPath();
            return false;
        }

        // Skip if already visited
        if (visited_.find(current) != visited_.end() &&
            current.x != maze_->GetStartPoint().x &&
            current.y != maze_->GetStartPoint().y) {
            return !stack_.empty();
        }

        // Mark as visited
        visited_.insert(current);

        if (maze_->GetCellState(current.x, current.y) != CellState::START &&
            maze_->GetCellState(current.x, current.y) != CellState::END) {
            maze_->SetCellState(current.x, current.y, CellState::VISITED);
        }

        // Debug output
        if (debugMode_) {
            std::cout << "Visiting: " << current.x << "," << current.y << std::endl;
            std::cout << "Walls: N=" << maze_->HasWall(current.x, current.y, 0)
                << " E=" << maze_->HasWall(current.x, current.y, 1)
                << " S=" << maze_->HasWall(current.x, current.y, 2)
                << " W=" << maze_->HasWall(current.x, current.y, 3) << std::endl;
        }

        // Try all directions (in reverse order for DFS to prefer certain directions)
        for (int i = 3; i >= 0; i--) {
            // Skip if there's a wall
            if (maze_->HasWall(current.x, current.y, i)) {
                if (debugMode_) {
                    std::cout << "Wall in direction " << i << std::endl;
                }
                continue;
            }

            Point next = { current.x + DIRECTIONS[i].x, current.y + DIRECTIONS[i].y };

            // Skip if not valid or already visited
            if (!maze_->IsValidCell(next.x, next.y) ||
                maze_->GetCellState(next.x, next.y) == CellState::WALL ||
                visited_.find(next) != visited_.end()) {
                continue;
            }

            // Add to visit and set parent for path reconstruction
            parents_[next] = current;
            stack_.push(next);

            if (debugMode_) {
                std::cout << "Pushed: " << next.x << "," << next.y << std::endl;
            }
        }

        return !stack_.empty();
    }

private:
    bool debugMode_ = false; // Set to true for debug output

    void Reset(Maze& maze) override {
        maze_ = &maze;

        // Clear previous data
        while (!stack_.empty()) stack_.pop();
        visited_.clear();
        parents_.clear();
        solution_.clear();
        solved_ = false;

        // Reset maze states except start and end
        for (int y = 0; y < maze.GetHeight(); y++) {
            for (int x = 0; x < maze.GetWidth(); x++) {
                if (maze.GetCellState(x, y) == CellState::VISITED ||
                    maze.GetCellState(x, y) == CellState::SOLUTION) {
                    maze.SetCellState(x, y, CellState::PATH);
                }
            }
        }

        // Start from the maze start point
        Point start = maze.GetStartPoint();
        maze.SetCellState(start.x, start.y, CellState::START);

        // Set end point
        Point end = maze.GetEndPoint();
        maze.SetCellState(end.x, end.y, CellState::END);

        // Initialize the search
        stack_.push(start);
        visited_.insert(start);
    }

    bool IsFinished() const override {
        return stack_.empty() || solved_;
    }

    bool IsSolved() const override {
        return solved_;
    }

private:
    void ReconstructPath() {
        solution_.clear();
        Point current = maze_->GetEndPoint();

        // Trace backwards from end to start
        while (current.x != maze_->GetStartPoint().x || current.y != maze_->GetStartPoint().y) {
            solution_.push_back(current);

            // Check if we have a valid parent
            if (parents_.find(current) == parents_.end()) {
                std::cout << "Error: Path reconstruction failed - broken path at " << current.x << "," << current.y << std::endl;
                break;
            }

            current = parents_[current];
        }

        // Add start point if needed
        if (solution_.empty() ||
            (solution_.back().x != maze_->GetStartPoint().x ||
                solution_.back().y != maze_->GetStartPoint().y)) {
            solution_.push_back(maze_->GetStartPoint());
        }

        // Mark the solution path
        for (const auto& point : solution_) {
            // Skip marking the end point
            if (point.x == maze_->GetEndPoint().x && point.y == maze_->GetEndPoint().y) {
                continue;
            }

            // Skip marking the start point
            if (point.x == maze_->GetStartPoint().x && point.y == maze_->GetStartPoint().y) {
                continue;
            }

            maze_->SetCellState(point.x, point.y, CellState::SOLUTION);
        }
    }

    Maze* maze_;
    std::stack<Point> stack_;
    std::unordered_set<Point> visited_;
    std::unordered_map<Point, Point> parents_;
    std::vector<Point> solution_;
    bool solved_ = false;
};

class BFSSolver : public MazeSolver {
public:
    void SolveMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (queue_.empty() || solved_) {
            return false;
        }

        Point current = queue_.front();
        queue_.pop();

        // If we've reached the end, reconstruct the path
        if (current.x == maze_->GetEndPoint().x && current.y == maze_->GetEndPoint().y) {
            solved_ = true;
            ReconstructPath();
            return false;
        }

        // Mark as visited if not already
        if (maze_->GetCellState(current.x, current.y) != CellState::VISITED &&
            maze_->GetCellState(current.x, current.y) != CellState::START) {
            maze_->SetCellState(current.x, current.y, CellState::VISITED);
        }

        // Try all directions
        for (int i = 0; i < 4; i++) {
            // Skip if there's a wall
            if (maze_->HasWall(current.x, current.y, i)) continue;

            Point next = { current.x + DIRECTIONS[i].x, current.y + DIRECTIONS[i].y };

            // Skip if already visited
            if (visited_.find(next) != visited_.end()) continue;

            // Add to visit and set parent for path reconstruction
            queue_.push(next);
            visited_.insert(next);
            parents_[next] = current;
        }

        return !queue_.empty();
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;

        // Clear previous data
        std::queue<Point> empty;
        std::swap(queue_, empty);
        visited_.clear();
        parents_.clear();
        solution_.clear();
        solved_ = false;

        // Reset maze states except start and end
        for (int y = 0; y < maze.GetHeight(); y++) {
            for (int x = 0; x < maze.GetWidth(); x++) {
                if (maze.GetCellState(x, y) == CellState::VISITED ||
                    maze.GetCellState(x, y) == CellState::SOLUTION) {
                    maze.SetCellState(x, y, CellState::PATH);
                }
            }
        }

        // Start from the maze start point
        Point start = maze.GetStartPoint();
        maze.SetCellState(start.x, start.y, CellState::START);

        // Set end point
        Point end = maze.GetEndPoint();
        maze.SetCellState(end.x, end.y, CellState::END);

        // Initialize the search
        queue_.push(start);
        visited_.insert(start);
    }

    bool IsFinished() const override {
        return queue_.empty() || solved_;
    }

    bool IsSolved() const override {
        return solved_;
    }

private:
    void ReconstructPath() {
        Point current = maze_->GetEndPoint();
        while (current.x != maze_->GetStartPoint().x || current.y != maze_->GetStartPoint().y) {
            solution_.push_back(current);
            if (parents_.find(current) == parents_.end()) break;
            current = parents_[current];
        }

        // Mark the solution path
        for (const auto& point : solution_) {
            if (point.x == maze_->GetEndPoint().x && point.y == maze_->GetEndPoint().y) continue;
            maze_->SetCellState(point.x, point.y, CellState::SOLUTION);
        }
    }

    Maze* maze_;
    std::queue<Point> queue_;
    std::unordered_set<Point> visited_;
    std::unordered_map<Point, Point> parents_;
    std::vector<Point> solution_;
    bool solved_ = false;
};

class AStarSolver : public MazeSolver {
public:
    void SolveMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (openSet_.empty() || solved_) {
            return false;
        }

        // Find the node with the lowest fScore
        Point current = GetNodeWithLowestFScore();

        // If we've reached the end, reconstruct the path
        if (current.x == maze_->GetEndPoint().x && current.y == maze_->GetEndPoint().y) {
            solved_ = true;
            ReconstructPath();
            return false;
        }

        // Remove current from open set and add to closed set
        openSet_.erase(std::find(openSet_.begin(), openSet_.end(), current));
        closedSet_.insert(current);

        // Mark as visited if not already
        if (maze_->GetCellState(current.x, current.y) != CellState::VISITED &&
            maze_->GetCellState(current.x, current.y) != CellState::START) {
            maze_->SetCellState(current.x, current.y, CellState::VISITED);
        }

        // Debug wall status around current cell
        if (isDebugMode_) {
            std::cout << "Cell " << current.x << "," << current.y << " walls: ";
            for (int i = 0; i < 4; i++) {
                std::cout << (maze_->HasWall(current.x, current.y, i) ? "1" : "0");
            }
            std::cout << std::endl;
        }

        // Try all directions
        for (int i = 0; i < 4; i++) {
            // Skip if there's a wall
            if (maze_->HasWall(current.x, current.y, i)) continue;

            Point neighbor = { current.x + DIRECTIONS[i].x, current.y + DIRECTIONS[i].y };

            // Skip if in closed set or not a valid cell
            if (!maze_->IsValidCell(neighbor.x, neighbor.y) ||
                closedSet_.find(neighbor) != closedSet_.end() ||
                maze_->GetCellState(neighbor.x, neighbor.y) == CellState::WALL) {
                continue;
            }

            // Calculate tentative gScore
            float tentativeGScore = gScores_[current] + 1;

            // Check if this path is better or if the neighbor is not in open set
            if (std::find(openSet_.begin(), openSet_.end(), neighbor) == openSet_.end() ||
                tentativeGScore < gScores_[neighbor]) {

                // Update the neighbor's data
                parents_[neighbor] = current;
                gScores_[neighbor] = tentativeGScore;
                fScores_[neighbor] = tentativeGScore + Heuristic(neighbor, maze_->GetEndPoint());

                // Add to open set if not already there
                if (std::find(openSet_.begin(), openSet_.end(), neighbor) == openSet_.end()) {
                    openSet_.push_back(neighbor);
                }
            }
        }

        return !openSet_.empty();
    }

private:
    bool isDebugMode_ = false;  // Set to true to enable debug output


    void Reset(Maze& maze) override {
        maze_ = &maze;

        // Clear previous data
        openSet_.clear();
        closedSet_.clear();
        parents_.clear();
        gScores_.clear();
        fScores_.clear();
        solution_.clear();
        solved_ = false;

        // Reset maze states except start and end
        for (int y = 0; y < maze.GetHeight(); y++) {
            for (int x = 0; x < maze.GetWidth(); x++) {
                if (maze.GetCellState(x, y) == CellState::VISITED ||
                    maze.GetCellState(x, y) == CellState::SOLUTION) {
                    maze.SetCellState(x, y, CellState::PATH);
                }
            }
        }

        // Start from the maze start point
        Point start = maze.GetStartPoint();
        maze.SetCellState(start.x, start.y, CellState::START);

        // Set end point
        Point end = maze.GetEndPoint();
        maze.SetCellState(end.x, end.y, CellState::END);

        // Initialize the search
        openSet_.push_back(start);
        gScores_[start] = 0;
        fScores_[start] = Heuristic(start, end);
    }

    bool IsFinished() const override {
        return openSet_.empty() || solved_;
    }

    bool IsSolved() const override {
        return solved_;
    }

private:
    float Heuristic(const Point& a, const Point& b) {
        // Manhattan distance
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    Point GetNodeWithLowestFScore() {
        float lowestScore = std::numeric_limits<float>::infinity();
        Point lowestNode = openSet_[0];

        for (const auto& node : openSet_) {
            if (fScores_[node] < lowestScore) {
                lowestScore = fScores_[node];
                lowestNode = node;
            }
        }

        return lowestNode;
    }

    void ReconstructPath() {
        Point current = maze_->GetEndPoint();
        while (current.x != maze_->GetStartPoint().x || current.y != maze_->GetStartPoint().y) {
            solution_.push_back(current);
            if (parents_.find(current) == parents_.end()) break;
            current = parents_[current];
        }

        // Mark the solution path
        for (const auto& point : solution_) {
            if (point.x == maze_->GetEndPoint().x && point.y == maze_->GetEndPoint().y) continue;
            maze_->SetCellState(point.x, point.y, CellState::SOLUTION);
        }
    }

    Maze* maze_;
    std::vector<Point> openSet_;
    std::unordered_set<Point> closedSet_;
    std::unordered_map<Point, Point> parents_;
    std::unordered_map<Point, float> gScores_;
    std::unordered_map<Point, float> fScores_;
    std::vector<Point> solution_;
    bool solved_ = false;
};

// ======= Application Class =======
class WilsonGenerator : public MazeGenerator {
public:
    void GenerateMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        // Safety check - limit the maximum steps to prevent hanging
        if (stepCount_++ > maxSteps_) {
            if (debugMode_) {
                std::cout << "Wilson: Maximum steps exceeded - finishing early" << std::endl;
            }
            FinishRemainingCells();
            return false;
        }

        // If all cells are processed, we're done
        if (unvisitedCells_.empty()) {
            return false;
        }

        // If we're currently performing a random walk
        if (inRandomWalk_) {
            walkStepCount_++;

            // If the walk is taking too long, abort and try a different cell
            if (walkStepCount_ > maxWalkSteps_) {
                if (debugMode_) {
                    std::cout << "Wilson: Random walk taking too long - aborting" << std::endl;
                }
                inRandomWalk_ = false;
                return true;
            }

            return PerformRandomWalkStep();
        }

        // Start a new random walk if we're not in one
        StartNewRandomWalk();
        return true;
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;
        maze.Reset();

        // Clear previous data
        mazeSet_.clear();
        unvisitedCells_.clear();
        currentPath_.clear();
        inRandomWalk_ = false;
        stepCount_ = 0;
        walkStepCount_ = 0;

        // Set maximum steps based on maze size for safety
        maxSteps_ = maze.GetWidth() * maze.GetHeight() * 10;
        maxWalkSteps_ = maze.GetWidth() * maze.GetHeight();

        // Create a list of all valid cells (use only odd coordinates)
        for (int y = 1; y < maze.GetHeight(); y += 2) {
            for (int x = 1; x < maze.GetWidth(); x += 2) {
                Point cell = { x, y };
                unvisitedCells_.push_back(cell);
            }
        }

        // Shuffle the list for randomness
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(unvisitedCells_.begin(), unvisitedCells_.end(), g);

        // Pick a random starting cell
        if (!unvisitedCells_.empty()) {
            Point start = unvisitedCells_.back();
            unvisitedCells_.pop_back();
            mazeSet_.insert(start);
            maze.CarvePathAt(start.x, start.y);

            // Make sure start and end points are included in the unvisited cells
            // so they get connected to the maze
            Point mazeStart = maze.GetStartPoint();
            Point mazeEnd = maze.GetEndPoint();

            // Only add if they're not already in our lists
            if (mazeSet_.find(mazeStart) == mazeSet_.end() &&
                std::find(unvisitedCells_.begin(), unvisitedCells_.end(), mazeStart) == unvisitedCells_.end()) {
                unvisitedCells_.push_back(mazeStart);
            }

            if (mazeSet_.find(mazeEnd) == mazeSet_.end() &&
                std::find(unvisitedCells_.begin(), unvisitedCells_.end(), mazeEnd) == unvisitedCells_.end()) {
                unvisitedCells_.push_back(mazeEnd);
            }

            if (debugMode_) {
                std::cout << "Wilson: Starting with cell " << start.x << "," << start.y << std::endl;
                std::cout << "Wilson: " << unvisitedCells_.size() << " cells remain to be visited" << std::endl;
                std::cout << "Wilson: Added start point " << mazeStart.x << "," << mazeStart.y
                    << " and end point " << mazeEnd.x << "," << mazeEnd.y << " to unvisited list" << std::endl;
            }
        }
    }

    bool IsFinished() const override {
        return unvisitedCells_.empty() || stepCount_ > maxSteps_;
    }

private:
    void StartNewRandomWalk() {
        if (unvisitedCells_.empty()) return;

        // Pick a random unvisited cell to start the walk
        int index = rand() % unvisitedCells_.size();
        currentCell_ = unvisitedCells_[index];

        // Initialize the path with the starting cell
        currentPath_.clear();
        currentPath_[currentCell_] = { -1, -1 }; // No direction for the starting cell

        // Reset walk step counter
        walkStepCount_ = 0;

        inRandomWalk_ = true;

        if (debugMode_) {
            std::cout << "Wilson: Starting new walk from " << currentCell_.x << "," << currentCell_.y << std::endl;
        }
    }

    bool PerformRandomWalkStep() {
        // Collect valid directions (ones that stay within the maze boundaries)
        std::vector<int> validDirections;
        for (int i = 0; i < 4; i++) {
            Point next = {
                currentCell_.x + DIRECTIONS[i].x * 2,
                currentCell_.y + DIRECTIONS[i].y * 2
            };

            if (maze_->IsValidCell(next.x, next.y)) {
                validDirections.push_back(i);
            }
        }

        // If no valid directions, abort this walk
        if (validDirections.empty()) {
            inRandomWalk_ = false;
            return true;
        }

        // Choose a random valid direction
        int dirIndex = rand() % validDirections.size();
        int direction = validDirections[dirIndex];

        Point next = {
            currentCell_.x + DIRECTIONS[direction].x * 2,
            currentCell_.y + DIRECTIONS[direction].y * 2
        };

        // If we've reached a cell that's already in the maze, add the entire path to the maze
        if (mazeSet_.find(next) != mazeSet_.end()) {
            AddPathToMaze(next);
            inRandomWalk_ = false;
            return true;
        }

        // Check if we've created a loop
        if (currentPath_.find(next) != currentPath_.end()) {
            // We've hit our own path - erase the loop
            EraseLoop(next);
        }
        else {
            // Add this step to our path
            currentPath_[next] = currentCell_;
        }

        // Move to the next cell
        currentCell_ = next;
        return true;
    }

    void EraseLoop(const Point& loopCell) {
        // Start at the current cell and work backwards until we hit the loop cell,
        // removing cells from the path as we go
        Point current = currentCell_;
        while (current.x != loopCell.x || current.y != loopCell.y) {
            if (currentPath_.find(current) == currentPath_.end()) {
                // Something went wrong, abort loop erasure
                if (debugMode_) {
                    std::cout << "Wilson: Error in loop erasure - aborting" << std::endl;
                }
                return;
            }

            Point prev = currentPath_[current];
            currentPath_.erase(current);
            current = prev;
        }
    }

    void AddPathToMaze(const Point& endCell) {
        // Connect the current path to the maze
        Point current = currentCell_;
        Point prev;

        while (true) {
            // Add the current cell to the maze
            mazeSet_.insert(current);
            maze_->CarvePathAt(current.x, current.y);

            // Remove from unvisited list
            auto it = std::find(unvisitedCells_.begin(), unvisitedCells_.end(), current);
            if (it != unvisitedCells_.end()) {
                // Swap and pop for efficient removal
                *it = unvisitedCells_.back();
                unvisitedCells_.pop_back();
            }

            // Check if we've reached the start of our path
            if (currentPath_.find(current) == currentPath_.end() ||
                currentPath_[current].x == -1) {
                // Now connect the last cell in our path to the end cell (which is already in the maze)
                ConnectCells(current, endCell);
                break;
            }

            // Get the previous cell in the path
            prev = currentPath_[current];

            // Connect the cells
            ConnectCells(prev, current);

            // Move to the previous cell
            current = prev;
        }
    }

    void ConnectCells(const Point& from, const Point& to) {
        // Find the direction from 'from' to 'to'
        int direction = -1;
        for (int i = 0; i < 4; i++) {
            if (from.x + DIRECTIONS[i].x * 2 == to.x &&
                from.y + DIRECTIONS[i].y * 2 == to.y) {
                direction = i;
                break;
            }
        }

        if (direction >= 0) {
            // Calculate the wall position
            Point wallPos = {
                from.x + DIRECTIONS[direction].x,
                from.y + DIRECTIONS[direction].y
            };

            // Carve the wall
            maze_->CarvePathAt(wallPos.x, wallPos.y);

            // Remove the walls in both cells
            maze_->RemoveWall(from.x, from.y, direction);
            maze_->RemoveWall(to.x, to.y, (direction + 2) % 4);
        }
    }

    // Handle any remaining unvisited cells by connecting them directly to the maze
    void FinishRemainingCells() {
        if (debugMode_) {
            std::cout << "Wilson: Finishing " << unvisitedCells_.size() << " remaining cells" << std::endl;
        }

        while (!unvisitedCells_.empty()) {
            Point cell = unvisitedCells_.back();
            unvisitedCells_.pop_back();

            // Carve this cell
            maze_->CarvePathAt(cell.x, cell.y);

            // Find the closest cell that's already in the maze
            Point closest;
            int closestDist = std::numeric_limits<int>::max();

            for (const Point& mazeCell : mazeSet_) {
                int dist = std::abs(cell.x - mazeCell.x) + std::abs(cell.y - mazeCell.y);
                if (dist < closestDist) {
                    closestDist = dist;
                    closest = mazeCell;
                }
            }

            // If we found a cell in the maze, connect to it
            if (closestDist != std::numeric_limits<int>::max()) {
                // Create a direct path to the closest cell
                while (cell.x != closest.x || cell.y != closest.y) {
                    // Determine direction to move
                    int dx = 0, dy = 0;
                    if (cell.x < closest.x) dx = 1;
                    else if (cell.x > closest.x) dx = -1;
                    else if (cell.y < closest.y) dy = 1;
                    else if (cell.y > closest.y) dy = -1;

                    // Calculate next cell and wall
                    Point next = { cell.x + dx * 2, cell.y + dy * 2 };
                    Point wall = { cell.x + dx, cell.y + dy };

                    // Carve the path
                    maze_->CarvePathAt(wall.x, wall.y);

                    // Find direction indices
                    int dirFromCell = -1, dirFromNext = -1;
                    for (int i = 0; i < 4; i++) {
                        if (DIRECTIONS[i].x == dx && DIRECTIONS[i].y == dy) {
                            dirFromCell = i;
                            dirFromNext = (i + 2) % 4;
                            break;
                        }
                    }

                    // Remove walls
                    if (dirFromCell >= 0 && maze_->IsValidCell(next.x, next.y)) {
                        maze_->RemoveWall(cell.x, cell.y, dirFromCell);
                        maze_->RemoveWall(next.x, next.y, dirFromNext);
                    }

                    // Move to next cell
                    cell = next;
                }
            }

            // Add to maze set
            mazeSet_.insert(cell);
        }
    }

    Maze* maze_;
    std::unordered_set<Point> mazeSet_;           // Cells that are part of the maze
    std::vector<Point> unvisitedCells_;           // Cells not yet part of the maze
    std::unordered_map<Point, Point> currentPath_; // Current random walk path
    Point currentCell_;                           // Current cell in the random walk
    bool inRandomWalk_ = false;                   // Whether we're currently performing a random walk
    bool debugMode_ = false;                      // Debug mode flag

    int stepCount_ = 0;                           // Counter for total steps
    int walkStepCount_ = 0;                       // Counter for steps in current walk
    int maxSteps_ = 10000;                        // Safety limit for total steps
    int maxWalkSteps_ = 1000;                     // Safety limit for a single walk
};

// Eller's Algorithm Implementation
class EllerGenerator : public MazeGenerator {
public:
    void GenerateMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (currentRow_ >= maze_->GetHeight()) {
            return false; // We've processed all rows
        }

        if (currentRow_ % 2 == 0) {
            // Skip even rows as they are wall rows
            currentRow_++;
            return true;
        }

        // Process the current row based on the stage
        switch (currentStage_) {
        case 0: // Initialize sets for this row if needed
            InitializeRowSets();
            currentStage_++;
            return true;

        case 1: // Randomly merge adjacent cells in the same row
            MergeAdjacentCells();
            currentStage_++;
            return true;

        case 2: // Create vertical connections to the next row
            CreateVerticalConnections();
            currentStage_++;
            return true;

        case 3: // Prepare for the next row
            if (currentRow_ == maze_->GetHeight() - 2) {
                // Last row - merge all remaining cells
                MergeLastRow();
            }

            currentRow_ += 2; // Move to the next odd row
            currentStage_ = 0; // Reset stage
            return true;
        }

        return false;
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;
        maze.Reset();

        // Clear previous data
        setMap_.clear();
        setParents_.clear();
        nextSet_ = 0;

        // Initialize for first row
        currentRow_ = 1; // Start with the first odd row
        currentStage_ = 0;
    }

    bool IsFinished() const override {
        return currentRow_ >= maze_->GetHeight();
    }

private:
    void InitializeRowSets() {
        // For each cell in the current row
        for (int x = 1; x < maze_->GetWidth(); x += 2) {
            Point cell = { x, currentRow_ };

            // If it's the first row or the cell doesn't have a set yet
            if (setMap_.find(cell) == setMap_.end()) {
                setMap_[cell] = nextSet_++;
                setParents_[setMap_[cell]] = setMap_[cell]; // Each set is its own parent initially
            }

            // Carve the path for this cell
            maze_->CarvePathAt(cell.x, cell.y);

            if (debugMode_) {
                std::cout << "Eller: Cell " << cell.x << "," << cell.y
                    << " in set " << setMap_[cell] << std::endl;
            }
        }
    }

    void MergeAdjacentCells() {
        // For each cell except the last in the current row
        for (int x = 1; x < maze_->GetWidth() - 2; x += 2) {
            // Randomly decide whether to merge with the next cell
            if (rand() % 2 == 0) {
                Point cell1 = { x, currentRow_ };
                Point cell2 = { x + 2, currentRow_ };

                // Only merge if they're in different sets
                int set1 = FindSet(cell1);
                int set2 = FindSet(cell2);

                if (set1 != set2) {
                    // Remove the wall between them
                    Point wall = { x + 1, currentRow_ };
                    maze_->CarvePathAt(wall.x, wall.y);

                    // Remove the internal walls
                    maze_->RemoveWall(cell1.x, cell1.y, 1); // East
                    maze_->RemoveWall(cell2.x, cell2.y, 3); // West

                    // Merge the sets
                    UnionSets(set1, set2);

                    if (debugMode_) {
                        std::cout << "Eller: Merged cells " << cell1.x << "," << cell1.y
                            << " and " << cell2.x << "," << cell2.y
                            << " (sets " << set1 << ", " << set2 << ")" << std::endl;
                    }
                }
            }
        }
    }

    void CreateVerticalConnections() {
        // Skip if we're at the last row
        if (currentRow_ >= maze_->GetHeight() - 2) {
            return;
        }

        // Track which sets have vertical connections
        std::unordered_set<int> setsWithConnections;

        // For each cell in the current row
        for (int x = 1; x < maze_->GetWidth(); x += 2) {
            Point cell = { x, currentRow_ };
            int set = FindSet(cell);

            // Randomly decide whether to create a vertical connection
            bool createConnection = (rand() % 2 == 0);

            // If this is the last chance for this set to have a connection, force one
            bool isLastCellInSet = true;
            for (int nextX = x + 2; nextX < maze_->GetWidth(); nextX += 2) {
                Point nextCell = { nextX, currentRow_ };
                if (FindSet(nextCell) == set) {
                    isLastCellInSet = false;
                    break;
                }
            }

            if (isLastCellInSet && setsWithConnections.find(set) == setsWithConnections.end()) {
                createConnection = true;
            }

            if (createConnection) {
                // Create a path to the cell below
                Point cellBelow = { x, currentRow_ + 2 };
                Point wall = { x, currentRow_ + 1 };

                // Carve the paths
                maze_->CarvePathAt(wall.x, wall.y);
                maze_->CarvePathAt(cellBelow.x, cellBelow.y);

                // Remove the internal walls
                maze_->RemoveWall(cell.x, cell.y, 2); // South
                maze_->RemoveWall(cellBelow.x, cellBelow.y, 0); // North

                // Assign the cell below to the same set
                setMap_[cellBelow] = set;

                // Mark this set as having a connection
                setsWithConnections.insert(set);

                if (debugMode_) {
                    std::cout << "Eller: Vertical connection from " << cell.x << "," << cell.y
                        << " to " << cellBelow.x << "," << cellBelow.y
                        << " (set " << set << ")" << std::endl;
                }
            }
        }
    }

    void MergeLastRow() {
        // For the last row, merge all adjacent cells that are in different sets
        for (int x = 1; x < maze_->GetWidth() - 2; x += 2) {
            Point cell1 = { x, currentRow_ };
            Point cell2 = { x + 2, currentRow_ };

            // Only merge if they're in different sets
            int set1 = FindSet(cell1);
            int set2 = FindSet(cell2);

            if (set1 != set2) {
                // Remove the wall between them
                Point wall = { x + 1, currentRow_ };
                maze_->CarvePathAt(wall.x, wall.y);

                // Remove the internal walls
                maze_->RemoveWall(cell1.x, cell1.y, 1); // East
                maze_->RemoveWall(cell2.x, cell2.y, 3); // West

                // Merge the sets
                UnionSets(set1, set2);

                if (debugMode_) {
                    std::cout << "Eller: Last row merged cells " << cell1.x << "," << cell1.y
                        << " and " << cell2.x << "," << cell2.y << std::endl;
                }
            }
        }
    }

    int FindSet(const Point& cell) {
        // If the cell doesn't have a set, give it a new one
        if (setMap_.find(cell) == setMap_.end()) {
            setMap_[cell] = nextSet_++;
            setParents_[setMap_[cell]] = setMap_[cell];
        }

        int setIndex = setMap_[cell];

        // Path compression
        if (setParents_[setIndex] != setIndex) {
            setParents_[setIndex] = FindSet(setParents_[setIndex]);
        }

        return setParents_[setIndex];
    }

    int FindSet(int setIndex) {
        // If the set doesn't have a parent, it is its own parent
        if (setParents_.find(setIndex) == setParents_.end()) {
            setParents_[setIndex] = setIndex;
        }

        // Path compression
        if (setParents_[setIndex] != setIndex) {
            setParents_[setIndex] = FindSet(setParents_[setIndex]);
        }

        return setParents_[setIndex];
    }

    void UnionSets(int set1, int set2) {
        // Merge set2 into set1
        setParents_[FindSet(set2)] = FindSet(set1);
    }

    Maze* maze_;
    std::unordered_map<Point, int> setMap_;       // Maps cells to their set ID
    std::unordered_map<int, int> setParents_;     // Disjoint-set data structure
    int nextSet_ = 0;                             // Next available set ID
    int currentRow_ = 1;                          // Current row being processed
    int currentStage_ = 0;                        // Current stage of processing
    bool debugMode_ = false;                      // Debug mode flag
};

// Growing Tree Algorithm Implementation
class GrowingTreeGenerator : public MazeGenerator {
public:
    enum class SelectionStrategy {
        NEWEST,    // Like DFS - pick the most recently added cell
        OLDEST,    // Like BFS - pick the oldest cell
        RANDOM,    // Random selection
        MIXED      // Mix of newest and random
    };

    GrowingTreeGenerator(SelectionStrategy strategy = SelectionStrategy::MIXED)
        : strategy_(strategy) {
    }

    void SetStrategy(SelectionStrategy strategy) {
        strategy_ = strategy;
    }

    void GenerateMaze(Maze& maze) override {
        Reset(maze);
        while (!IsFinished()) {
            Step();
        }
    }

    bool Step() override {
        if (activeCells_.empty()) {
            return false;
        }

        // Choose a cell from the active list based on the strategy
        int index = 0;
        switch (strategy_) {
        case SelectionStrategy::NEWEST:
            index = activeCells_.size() - 1;
            break;

        case SelectionStrategy::OLDEST:
            index = 0;
            break;

        case SelectionStrategy::RANDOM:
            index = rand() % activeCells_.size();
            break;

        case SelectionStrategy::MIXED:
            // 50% chance to pick newest, 50% chance for random
            if (rand() % 2 == 0) {
                index = activeCells_.size() - 1;
            }
            else {
                index = rand() % activeCells_.size();
            }
            break;
        }

        Point current = activeCells_[index];

        // Find unvisited neighbors
        std::vector<std::pair<Point, int>> unvisitedNeighbors;
        for (int i = 0; i < 4; i++) {
            Point neighbor = { current.x + DIRECTIONS[i].x * 2, current.y + DIRECTIONS[i].y * 2 };
            if (maze_->IsValidCell(neighbor.x, neighbor.y) &&
                maze_->GetCellState(neighbor.x, neighbor.y) == CellState::WALL) {
                unvisitedNeighbors.push_back({ neighbor, i });
            }
        }

        if (!unvisitedNeighbors.empty()) {
            // Choose a random unvisited neighbor
            int randomNeighbor = rand() % unvisitedNeighbors.size();
            Point next = unvisitedNeighbors[randomNeighbor].first;
            int direction = unvisitedNeighbors[randomNeighbor].second;

            // Carve a path to the neighbor
            Point wall = { current.x + DIRECTIONS[direction].x, current.y + DIRECTIONS[direction].y };
            maze_->CarvePathAt(wall.x, wall.y);
            maze_->CarvePathAt(next.x, next.y);

            // Remove the walls
            maze_->RemoveWall(current.x, current.y, direction);
            maze_->RemoveWall(next.x, next.y, (direction + 2) % 4);

            // Add the neighbor to the active list
            activeCells_.push_back(next);

            if (debugMode_) {
                std::cout << "GrowingTree: Connected " << current.x << "," << current.y
                    << " to " << next.x << "," << next.y << std::endl;
            }
        }
        else {
            // No unvisited neighbors, remove this cell from the active list
            activeCells_.erase(activeCells_.begin() + index);

            if (debugMode_) {
                std::cout << "GrowingTree: Removed cell " << current.x << "," << current.y
                    << " from active list" << std::endl;
            }
        }

        return !activeCells_.empty();
    }

    void Reset(Maze& maze) override {
        maze_ = &maze;
        maze.Reset();

        // Clear the active list
        activeCells_.clear();

        // Start with a random cell
        int startX = 1 + (rand() % ((maze.GetWidth() - 2) / 2)) * 2;
        int startY = 1 + (rand() % ((maze.GetHeight() - 2) / 2)) * 2;

        // Ensure coordinates are odd
        if (startX % 2 == 0) startX--;
        if (startY % 2 == 0) startY--;

        // Make sure start is in bounds
        startX = std::max(1, std::min(startX, maze.GetWidth() - 2));
        startY = std::max(1, std::min(startY, maze.GetHeight() - 2));

        Point start = { startX, startY };
        maze.CarvePathAt(start.x, start.y);
        activeCells_.push_back(start);

        if (debugMode_) {
            std::cout << "GrowingTree: Started at " << startX << "," << startY
                << " with strategy " << static_cast<int>(strategy_) << std::endl;
        }
    }

    bool IsFinished() const override {
        return activeCells_.empty();
    }

private:
    Maze* maze_;
    std::vector<Point> activeCells_; // List of active cells
    SelectionStrategy strategy_;     // Cell selection strategy
    bool debugMode_ = false;         // Debug mode flag
};

class MazeApp {
public:
    MazeApp()
        : maze_(DEFAULT_MAZE_WIDTH, DEFAULT_MAZE_HEIGHT),
        isGenerating_(false),
        isSolving_(false),
        generationSpeed_(10),
        solvingSpeed_(10),
        selectedGenerationAlgo_(GenerationAlgorithm::RECURSIVE_BACKTRACKER),
        selectedSolvingAlgo_(SolvingAlgorithm::DFS),
        growingTreeStrategy_(GrowingTreeGenerator::SelectionStrategy::MIXED) {

        // Initialize generators
        generators_[GenerationAlgorithm::RECURSIVE_BACKTRACKER] = std::make_unique<RecursiveBacktrackerGenerator>();
        generators_[GenerationAlgorithm::KRUSKAL] = std::make_unique<KruskalGenerator>();
        generators_[GenerationAlgorithm::PRIM] = std::make_unique<PrimGenerator>();
        generators_[GenerationAlgorithm::WILSON] = std::make_unique<WilsonGenerator>();
        generators_[GenerationAlgorithm::ELLER] = std::make_unique<EllerGenerator>();
        generators_[GenerationAlgorithm::GROWING_TREE] = std::make_unique<GrowingTreeGenerator>(growingTreeStrategy_);

        // Initialize solvers
        solvers_[SolvingAlgorithm::DFS] = std::make_unique<DFSSolver>();
        solvers_[SolvingAlgorithm::BFS] = std::make_unique<BFSSolver>();
        solvers_[SolvingAlgorithm::A_STAR] = std::make_unique<AStarSolver>();

        // Initialize SDL
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
            exit(1);
        }

        // Create window
        window_ = SDL_CreateWindow("Maze Generator & Solver",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            SCREEN_WIDTH, SCREEN_HEIGHT,
            SDL_WINDOW_SHOWN);
        if (!window_) {
            std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
            SDL_Quit();
            exit(1);
        }

        // Create renderer
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!renderer_) {
            SDL_DestroyWindow(window_);
            std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
            SDL_Quit();
            exit(1);
        }

        // Initialize ImGui
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;

        // Setup ImGui style
        ImGui::StyleColorsDark();

        // Setup Platform/Renderer backends
        ImGui_ImplSDL2_InitForSDLRenderer(window_, renderer_);
        ImGui_ImplSDLRenderer2_Init(renderer_);

        // Initialize maze
        ResetMaze();
    }

    ~MazeApp() {
        // Cleanup ImGui
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        // Cleanup SDL
        SDL_DestroyRenderer(renderer_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    void Run() {
        bool running = true;
        SDL_Event event;

        while (running) {
            // Handle events
            while (SDL_PollEvent(&event)) {
                ImGui_ImplSDL2_ProcessEvent(&event);

                if (event.type == SDL_QUIT) {
                    running = false;
                }
            }

            // Start ImGui frame
            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();

            // Update maze generation/solving
            UpdateMaze();

            // Create ImGui window
            CreateUI();

            // Render
            Render();
        }
    }

private:
    void ResetMaze() {
        maze_.Reset();
        isGenerating_ = false;
        isSolving_ = false;
    }

    void GenerateMaze() {
        ResetMaze();
        generators_[selectedGenerationAlgo_]->Reset(maze_);
        isGenerating_ = true;

        // Debug output of maze structure before generation
        if (debugMode_) {
            std::cout << "Maze structure after reset (before generation):" << std::endl;
            for (int y = 0; y < maze_.GetHeight(); y++) {
                for (int x = 0; x < maze_.GetWidth(); x++) {
                    char c = maze_.GetCellState(x, y) == CellState::WALL ? '#' : ' ';
                    std::cout << c;
                }
                std::cout << std::endl;
            }
        }
    }

private:
    bool debugMode_ = false; // Set to true for debugging output

    void SolveMaze() {
        solvers_[selectedSolvingAlgo_]->Reset(maze_);
        isSolving_ = true;
    }

    void UpdateMaze() {
        // Handle maze generation
        if (isGenerating_) {
            for (int i = 0; i < generationSpeed_ && isGenerating_; i++) {
                if (!generators_[selectedGenerationAlgo_]->Step()) {
                    isGenerating_ = false;

                    // Mark start and end points
                    maze_.SetCellState(maze_.GetStartPoint().x, maze_.GetStartPoint().y, CellState::START);
                    maze_.SetCellState(maze_.GetEndPoint().x, maze_.GetEndPoint().y, CellState::END);

                    // Verify wall connections after generation (debug)
                    if (debugMode_) {
                        std::cout << "Maze generation complete. Verifying paths..." << std::endl;
                        VerifyMazeConnectivity();
                    }
                }
            }
        }

        // Handle maze solving
        if (isSolving_) {
            for (int i = 0; i < solvingSpeed_ && isSolving_; i++) {
                if (!solvers_[selectedSolvingAlgo_]->Step()) {
                    isSolving_ = false;

                    if (debugMode_) {
                        if (solvers_[selectedSolvingAlgo_]->IsSolved()) {
                            std::cout << "Maze solved successfully!" << std::endl;
                        }
                        else {
                            std::cout << "Solver could not find a path!" << std::endl;
                        }
                    }
                }
            }
        }
    }

    // Verify that the maze has proper connectivity
    void VerifyMazeConnectivity() {
        for (int y = 1; y < maze_.GetHeight() - 1; y++) {
            for (int x = 1; x < maze_.GetWidth() - 1; x++) {
                if (maze_.GetCellState(x, y) != CellState::WALL) {
                    // Check for proper wall flags
                    std::cout << "Cell at " << x << "," << y << " walls: ";
                    for (int i = 0; i < 4; i++) {
                        std::cout << (maze_.HasWall(x, y, i) ? "1" : "0");
                    }
                    std::cout << std::endl;

                    // Check for consistency between adjacent cells
                    for (int i = 0; i < 4; i++) {
                        Point neighbor = { x + DIRECTIONS[i].x, y + DIRECTIONS[i].y };
                        if (maze_.IsValidCell(neighbor.x, neighbor.y) &&
                            maze_.GetCellState(neighbor.x, neighbor.y) != CellState::WALL) {

                            bool hasWall = maze_.HasWall(x, y, i);
                            int oppositeDir = (i + 2) % 4;
                            bool neighborHasWall = maze_.HasWall(neighbor.x, neighbor.y, oppositeDir);

                            if (hasWall != neighborHasWall) {
                                std::cout << "Wall inconsistency at " << x << "," << y
                                    << " dir " << i << " with neighbor "
                                    << neighbor.x << "," << neighbor.y << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }

    void CreateUI() {
        ImGui::Begin("Maze Controls");

        // Maze dimensions
        static int mazeWidth = DEFAULT_MAZE_WIDTH;
        static int mazeHeight = DEFAULT_MAZE_HEIGHT;

        ImGui::Text("Maze Dimensions");
        ImGui::InputInt("Width", &mazeWidth);
        ImGui::InputInt("Height", &mazeHeight);

        // Clamp to reasonable values
        mazeWidth = std::max(5, std::min(mazeWidth, 201));
        mazeHeight = std::max(5, std::min(mazeHeight, 1371));

        // Make sure dimensions are odd
        if (mazeWidth % 2 == 0) mazeWidth++;
        if (mazeHeight % 2 == 0) mazeHeight++;

        // Apply button
        if (ImGui::Button("Apply Dimensions")) {
            maze_ = Maze(mazeWidth, mazeHeight);
            ResetMaze();
        }

        ImGui::Separator();

        // Generation Algorithm
        ImGui::Text("Generation Algorithm");

        if (ImGui::RadioButton("Recursive Backtracker", selectedGenerationAlgo_ == GenerationAlgorithm::RECURSIVE_BACKTRACKER)) {
            selectedGenerationAlgo_ = GenerationAlgorithm::RECURSIVE_BACKTRACKER;
        }
        if (ImGui::RadioButton("Kruskal's Algorithm", selectedGenerationAlgo_ == GenerationAlgorithm::KRUSKAL)) {
            selectedGenerationAlgo_ = GenerationAlgorithm::KRUSKAL;
        }
        if (ImGui::RadioButton("Prim's Algorithm", selectedGenerationAlgo_ == GenerationAlgorithm::PRIM)) {
            selectedGenerationAlgo_ = GenerationAlgorithm::PRIM;
        }
        if (ImGui::RadioButton("Wilson's Algorithm", selectedGenerationAlgo_ == GenerationAlgorithm::WILSON)) {
            selectedGenerationAlgo_ = GenerationAlgorithm::WILSON;
        }
        if (ImGui::RadioButton("Eller's Algorithm", selectedGenerationAlgo_ == GenerationAlgorithm::ELLER)) {
            selectedGenerationAlgo_ = GenerationAlgorithm::ELLER;
        }
        if (ImGui::RadioButton("Growing Tree Algorithm", selectedGenerationAlgo_ == GenerationAlgorithm::GROWING_TREE)) {
            selectedGenerationAlgo_ = GenerationAlgorithm::GROWING_TREE;
        }

        // Growing Tree strategy selection (only show if Growing Tree is selected)
        if (selectedGenerationAlgo_ == GenerationAlgorithm::GROWING_TREE) {
            ImGui::Indent();
            ImGui::Text("Cell Selection Strategy:");

            // Cast to int for ImGui radio buttons
            int strategy = static_cast<int>(growingTreeStrategy_);

            if (ImGui::RadioButton("Newest (DFS-like)", strategy == 0)) {
                strategy = 0;
                growingTreeStrategy_ = GrowingTreeGenerator::SelectionStrategy::NEWEST;
                UpdateGrowingTreeStrategy();
            }
            if (ImGui::RadioButton("Oldest (BFS-like)", strategy == 1)) {
                strategy = 1;
                growingTreeStrategy_ = GrowingTreeGenerator::SelectionStrategy::OLDEST;
                UpdateGrowingTreeStrategy();
            }
            if (ImGui::RadioButton("Random", strategy == 2)) {
                strategy = 2;
                growingTreeStrategy_ = GrowingTreeGenerator::SelectionStrategy::RANDOM;
                UpdateGrowingTreeStrategy();
            }
            if (ImGui::RadioButton("Mixed (Newest/Random)", strategy == 3)) {
                strategy = 3;
                growingTreeStrategy_ = GrowingTreeGenerator::SelectionStrategy::MIXED;
                UpdateGrowingTreeStrategy();
            }

            ImGui::Unindent();
        }

        ImGui::Text("Generation Speed");
        ImGui::SliderInt("##GenSpeed", &generationSpeed_, 1, 50);

        // Generate button
        if (ImGui::Button("Generate Maze") && !isGenerating_ && !isSolving_) {
            GenerateMaze();
        }

        ImGui::SameLine();

        // Reset button
        if (ImGui::Button("Reset Maze") && !isGenerating_ && !isSolving_) {
            ResetMaze();
        }

        ImGui::Separator();

        // Solving Algorithm
        ImGui::Text("Solving Algorithm");

        if (ImGui::RadioButton("Depth-First Search", selectedSolvingAlgo_ == SolvingAlgorithm::DFS)) {
            selectedSolvingAlgo_ = SolvingAlgorithm::DFS;
        }
        if (ImGui::RadioButton("Breadth-First Search", selectedSolvingAlgo_ == SolvingAlgorithm::BFS)) {
            selectedSolvingAlgo_ = SolvingAlgorithm::BFS;
        }
        if (ImGui::RadioButton("A* Search", selectedSolvingAlgo_ == SolvingAlgorithm::A_STAR)) {
            selectedSolvingAlgo_ = SolvingAlgorithm::A_STAR;
        }

        ImGui::Text("Solving Speed");
        ImGui::SliderInt("##SolveSpeed", &solvingSpeed_, 1, 50);

        // Solve button
        bool canSolve = !isGenerating_ && !isSolving_ &&
            (maze_.GetCellState(maze_.GetStartPoint().x, maze_.GetStartPoint().y) == CellState::START);

        if (!canSolve) {
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            ImGui::Button("Solve Maze");
            ImGui::PopStyleVar();

            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Generate a maze first");
            }
        }
        else {
            if (ImGui::Button("Solve Maze")) {
                SolveMaze();
            }
        }

        ImGui::End();
    }

    void UpdateGrowingTreeStrategy() {
        // Update the generator if it exists
        auto it = generators_.find(GenerationAlgorithm::GROWING_TREE);
        if (it != generators_.end()) {
            dynamic_cast<GrowingTreeGenerator*>(it->second.get())->SetStrategy(growingTreeStrategy_);
        }
    }

    void DrawMaze() {
        // Calculate centered position for the maze
        int mazePixelWidth = maze_.GetWidth() * CELL_SIZE;
        int mazePixelHeight = maze_.GetHeight() * CELL_SIZE;

        int startX = (SCREEN_WIDTH - mazePixelWidth) / 2;
        int startY = (SCREEN_HEIGHT - mazePixelHeight) / 2;

        // Draw cells
        for (int y = 0; y < maze_.GetHeight(); y++) {
            for (int x = 0; x < maze_.GetWidth(); x++) {
                int cellX = startX + x * CELL_SIZE;
                int cellY = startY + y * CELL_SIZE;

                SDL_Rect cellRect = { cellX, cellY, CELL_SIZE, CELL_SIZE };

                // Set color based on cell state
                SDL_Color cellColor;
                switch (maze_.GetCellState(x, y)) {
                case CellState::WALL:
                    cellColor = BLACK;
                    break;
                case CellState::PATH:
                    cellColor = WHITE;
                    break;
                case CellState::VISITED:
                    cellColor = BLUE;
                    break;
                case CellState::CURRENT:
                    cellColor = YELLOW;
                    break;
                case CellState::START:
                    cellColor = GREEN;
                    break;
                case CellState::END:
                    cellColor = RED;
                    break;
                case CellState::SOLUTION:
                    cellColor = PURPLE;
                    break;
                }

                // Fill the cell with its background color
                SDL_SetRenderDrawColor(renderer_, cellColor.r, cellColor.g, cellColor.b, cellColor.a);
                SDL_RenderFillRect(renderer_, &cellRect);

                // Draw grid lines for debugging (optional)
                bool showGridLines = false;
                if (showGridLines) {
                    SDL_SetRenderDrawColor(renderer_, 128, 128, 128, 255); // Gray for grid
                    SDL_Rect gridRect = { cellX, cellY, CELL_SIZE, CELL_SIZE };
                    SDL_RenderDrawRect(renderer_, &gridRect);
                }

                // Draw walls if this is not a wall cell
                if (maze_.GetCellState(x, y) != CellState::WALL) {
                    SDL_SetRenderDrawColor(renderer_, BLACK.r, BLACK.g, BLACK.b, BLACK.a);

                    // North wall
                    if (maze_.HasWall(x, y, 0)) {
                        SDL_Rect wallRect = { cellX, cellY, CELL_SIZE, WALL_THICKNESS };
                        SDL_RenderFillRect(renderer_, &wallRect);
                    }

                    // East wall
                    if (maze_.HasWall(x, y, 1)) {
                        SDL_Rect wallRect = { cellX + CELL_SIZE - WALL_THICKNESS, cellY, WALL_THICKNESS, CELL_SIZE };
                        SDL_RenderFillRect(renderer_, &wallRect);
                    }

                    // South wall
                    if (maze_.HasWall(x, y, 2)) {
                        SDL_Rect wallRect = { cellX, cellY + CELL_SIZE - WALL_THICKNESS, CELL_SIZE, WALL_THICKNESS };
                        SDL_RenderFillRect(renderer_, &wallRect);
                    }

                    // West wall
                    if (maze_.HasWall(x, y, 3)) {
                        SDL_Rect wallRect = { cellX, cellY, WALL_THICKNESS, CELL_SIZE };
                        SDL_RenderFillRect(renderer_, &wallRect);
                    }
                }
            }
        }
    }

    void Render() {
        // Clear screen
        SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255);
        SDL_RenderClear(renderer_);

        // Draw maze
        DrawMaze();

        // Render ImGui
        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());

        // Present
        SDL_RenderPresent(renderer_);
    }

    Maze maze_;
    SDL_Window* window_;
    SDL_Renderer* renderer_;

    std::unordered_map<GenerationAlgorithm, std::unique_ptr<MazeGenerator>> generators_;
    std::unordered_map<SolvingAlgorithm, std::unique_ptr<MazeSolver>> solvers_;

    GenerationAlgorithm selectedGenerationAlgo_;
    SolvingAlgorithm selectedSolvingAlgo_;
    GrowingTreeGenerator::SelectionStrategy growingTreeStrategy_;

    bool isGenerating_;
    bool isSolving_;
    int generationSpeed_;
    int solvingSpeed_;
};

int main(int argc, char* argv[]) {
    // Seed random number generator
    srand(static_cast<unsigned int>(time(nullptr)));

    // Create and run the application
    MazeApp app;
    app.Run();

    return 0;
}