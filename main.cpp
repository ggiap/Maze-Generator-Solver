#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <random>
#include <ctime>
#include <algorithm>
#include <map>
#include <set>
#include <SDL2/SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

// Cell structure representing a single cell in the maze
struct Cell {
    bool visited = false;
    bool walls[4] = { true, true, true, true }; // top, right, bottom, left
};

// For Kruskal's algorithm
struct Edge {
    int x1, y1, x2, y2;

    Edge(int _x1, int _y1, int _x2, int _y2)
        : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {
    }
};

// For Disjoint-set data structure (used in Kruskal's algorithm)
class DisjointSet {
private:
    std::vector<int> parent;
    std::vector<int> rank;

public:
    DisjointSet(int size) {
        parent.resize(size);
        rank.resize(size, 0);

        // Initialize each element as its own parent
        for (int i = 0; i < size; i++) {
            parent[i] = i;
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]); // Path compression
        }
        return parent[x];
    }

    void unionSets(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);

        if (rootX == rootY) return;

        // Union by rank
        if (rank[rootX] < rank[rootY]) {
            parent[rootX] = rootY;
        }
        else if (rank[rootX] > rank[rootY]) {
            parent[rootY] = rootX;
        }
        else {
            parent[rootY] = rootX;
            rank[rootX]++;
        }
    }
};

enum class Algorithm {
    RECURSIVE_BACKTRACKING,
    PRIMS,
    KRUSKALS,
    ELLERS
};

class MazeGenerator {
private:
    int width, height;
    std::vector<std::vector<Cell>> grid;
    std::mt19937 rng;

    // SDL variables
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    const int cellSize = 20;
    const int wallThickness = 2;

    // ImGui variables
    bool showUI = true;
    Algorithm currentAlgorithm = Algorithm::RECURSIVE_BACKTRACKING;
    bool generationInProgress = false;
    bool autoGenerate = false;
    int generationSpeed = 10; // milliseconds delay

    // For tracking generation progress
    std::stack<std::pair<int, int>> backtrackStack;
    std::vector<Edge> kruskalEdges;
    int kruskalCurrentEdge = 0;
    std::vector<std::pair<int, int>> primFrontier;

    // For Eller's algorithm
    int ellerCurrentRow = 0;
    std::vector<int> ellerSets;  // set ID for each cell in current row
    int ellerNextSetId = 1;      // next available set ID

    // For tracking solver
    bool solverEnabled = false;
    std::pair<int, int> startPoint;
    std::pair<int, int> endPoint;
    std::vector<std::pair<int, int>> solutionPath;
    bool solutionFound = false;

public:
    MazeGenerator(int w, int h) : width(w), height(h) {
        // Initialize the grid
        grid.resize(height, std::vector<Cell>(width));

        // Seed the random number generator
        rng.seed(static_cast<unsigned int>(std::time(nullptr)));

        // Initialize SDL
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
            exit(1);
        }

        // Create window
        window = SDL_CreateWindow("Maze Generator with ImGui",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            width * cellSize + 300, // Extra space for ImGui
            height * cellSize,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (!window) {
            std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
            exit(1);
        }

        // Create renderer
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer) {
            std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
            exit(1);
        }

        // Setup ImGui
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;

        // Setup ImGui style
        ImGui::StyleColorsDark();

        // Setup Platform/Renderer backends
        ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
        ImGui_ImplSDLRenderer2_Init(renderer);
    }

    ~MazeGenerator() {
        // Cleanup ImGui
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        // Clean up SDL resources
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    // Check if coordinates are within the grid
    bool isValid(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // Remove wall between two cells
    void removeWall(int x1, int y1, int x2, int y2) {
        if (x1 == x2) {
            if (y1 > y2) {
                grid[y1][x1].walls[0] = false; // Remove top wall of (x1, y1)
                grid[y2][x2].walls[2] = false; // Remove bottom wall of (x2, y2)
            }
            else {
                grid[y1][x1].walls[2] = false; // Remove bottom wall of (x1, y1)
                grid[y2][x2].walls[0] = false; // Remove top wall of (x2, y2)
            }
        }
        else {
            if (x1 > x2) {
                grid[y1][x1].walls[3] = false; // Remove left wall of (x1, y1)
                grid[y2][x2].walls[1] = false; // Remove right wall of (x2, y2)
            }
            else {
                grid[y1][x1].walls[1] = false; // Remove right wall of (x1, y1)
                grid[y2][x2].walls[3] = false; // Remove left wall of (x2, y2)
            }
        }
    }

    // Reset the maze grid
    void resetMaze() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                grid[y][x].visited = false;
                grid[y][x].walls[0] = true; // top
                grid[y][x].walls[1] = true; // right
                grid[y][x].walls[2] = true; // bottom
                grid[y][x].walls[3] = true; // left
            }
        }

        // Reset algorithm-specific variables
        backtrackStack = std::stack<std::pair<int, int>>();
        kruskalEdges.clear();
        kruskalCurrentEdge = 0;
        primFrontier.clear();
        ellerCurrentRow = 0;
        ellerSets.clear();
        ellerNextSetId = 1;

        // Reset solver
        solverEnabled = false;
        solutionPath.clear();
        solutionFound = false;

        generationInProgress = false;
    }

    // Initialize maze generation based on selected algorithm
    void initMazeGeneration() {
        resetMaze();

        // Directions: right, down, left, up
        const int dx[] = { 1, 0, -1, 0 };
        const int dy[] = { 0, 1, 0, -1 };

        generationInProgress = true;

        switch (currentAlgorithm) {
        case Algorithm::RECURSIVE_BACKTRACKING: {
            // Start at a random cell
            int startX = rng() % width;
            int startY = rng() % height;

            backtrackStack.push({ startX, startY });
            grid[startY][startX].visited = true;
            break;
        }

        case Algorithm::PRIMS: {
            // Start with a random cell
            int startX = rng() % width;
            int startY = rng() % height;
            grid[startY][startX].visited = true;

            // Add walls from the starting cell to the frontier
            for (int i = 0; i < 4; i++) {
                int nx = startX + dx[i];
                int ny = startY + dy[i];

                if (isValid(nx, ny) && !grid[ny][nx].visited) {
                    primFrontier.push_back({ nx, ny });
                }
            }
            break;
        }

        case Algorithm::KRUSKALS: {
            // Create a list of all walls
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    // Add right wall if not at the right edge
                    if (x < width - 1) {
                        kruskalEdges.push_back(Edge(x, y, x + 1, y));
                    }

                    // Add bottom wall if not at the bottom edge
                    if (y < height - 1) {
                        kruskalEdges.push_back(Edge(x, y, x, y + 1));
                    }
                }
            }

            // Shuffle the edges
            std::shuffle(kruskalEdges.begin(), kruskalEdges.end(), rng);
            kruskalCurrentEdge = 0;
            break;
        }

        case Algorithm::ELLERS: {
            // Initialize first row: each cell gets its own set
            ellerSets.resize(width);
            for (int x = 0; x < width; x++) {
                ellerSets[x] = ellerNextSetId++;
            }
            ellerCurrentRow = 0;
            break;
        }
        }
    }

    // Perform one step of the current algorithm
    bool performGenerationStep() {
        if (!generationInProgress) {
            return false;
        }

        // Directions: right, down, left, up
        const int dx[] = { 1, 0, -1, 0 };
        const int dy[] = { 0, 1, 0, -1 };

        bool madeProgress = false;

        switch (currentAlgorithm) {
        case Algorithm::RECURSIVE_BACKTRACKING: {
            if (backtrackStack.empty()) {
                generationInProgress = false;
                return false;
            }

            int x = backtrackStack.top().first;
            int y = backtrackStack.top().second;

            // Find unvisited neighbors
            std::vector<int> neighbors;
            for (int i = 0; i < 4; i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (isValid(nx, ny) && !grid[ny][nx].visited) {
                    neighbors.push_back(i);
                }
            }

            if (!neighbors.empty()) {
                // Choose a random neighbor
                int nextDir = neighbors[rng() % neighbors.size()];
                int nx = x + dx[nextDir];
                int ny = y + dy[nextDir];

                // Remove the wall between current cell and chosen neighbor
                removeWall(x, y, nx, ny);

                // Mark the neighbor as visited and push it to the stack
                grid[ny][nx].visited = true;
                backtrackStack.push({ nx, ny });
                madeProgress = true;
            }
            else {
                // Backtrack
                backtrackStack.pop();
                madeProgress = true;
            }
            break;
        }

        case Algorithm::PRIMS: {
            if (primFrontier.empty()) {
                generationInProgress = false;
                return false;
            }

            // Choose a random cell from the frontier
            int index = rng() % primFrontier.size();
            int x = primFrontier[index].first;
            int y = primFrontier[index].second;

            // Find all visited neighbors
            std::vector<std::pair<int, int>> visitedNeighbors;
            for (int i = 0; i < 4; i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (isValid(nx, ny) && grid[ny][nx].visited) {
                    visitedNeighbors.push_back({ nx, ny });
                }
            }

            // Choose a random visited neighbor
            if (!visitedNeighbors.empty()) {
                int nIndex = rng() % visitedNeighbors.size();
                int nx = visitedNeighbors[nIndex].first;
                int ny = visitedNeighbors[nIndex].second;

                // Remove the wall between the frontier cell and the chosen visited neighbor
                removeWall(x, y, nx, ny);

                // Mark the frontier cell as visited
                grid[y][x].visited = true;

                // Remove this cell from the frontier
                primFrontier[index] = primFrontier.back();
                primFrontier.pop_back();

                // Add unvisited neighbors to the frontier
                for (int i = 0; i < 4; i++) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    if (isValid(nx, ny) && !grid[ny][nx].visited) {
                        // Check if the cell is already in the frontier
                        bool inFrontier = false;
                        for (const auto& cell : primFrontier) {
                            if (cell.first == nx && cell.second == ny) {
                                inFrontier = true;
                                break;
                            }
                        }

                        if (!inFrontier) {
                            primFrontier.push_back({ nx, ny });
                        }
                    }
                }

                madeProgress = true;
            }
            break;
        }

        case Algorithm::KRUSKALS: {
            if (kruskalCurrentEdge >= kruskalEdges.size()) {
                generationInProgress = false;
                return false;
            }

            // Get current edge
            Edge& edge = kruskalEdges[kruskalCurrentEdge++];

            // Calculate cell indices for disjoint set
            int index1 = edge.y1 * width + edge.x1;
            int index2 = edge.y2 * width + edge.x2;

            // Create disjoint set for the entire grid
            static DisjointSet* dset = nullptr;
            if (dset == nullptr) {
                dset = new DisjointSet(width * height);
            }

            // If cells are not in the same set, add the edge and unite the sets
            if (dset->find(index1) != dset->find(index2)) {
                removeWall(edge.x1, edge.y1, edge.x2, edge.y2);
                dset->unionSets(index1, index2);

                // Mark cells as visited for visualization
                grid[edge.y1][edge.x1].visited = true;
                grid[edge.y2][edge.x2].visited = true;

                madeProgress = true;
            }

            // If we've finished, clean up
            if (kruskalCurrentEdge >= kruskalEdges.size()) {
                delete dset;
                dset = nullptr;
            }

            break;
        }

        case Algorithm::ELLERS: {
            // Eller's algorithm processes the maze row by row
            if (ellerCurrentRow >= height) {
                generationInProgress = false;
                return false;
            }

            // Phase 1: Randomly join adjacent cells in the current row if they're in different sets
            if (ellerCurrentRow < height - 1) {  // Not the last row
                for (int x = 0; x < width - 1; x++) {
                    // Randomly decide if we should remove the wall between cells
                    if (rng() % 2 == 0 && ellerSets[x] != ellerSets[x + 1]) {
                        // Remove the right wall (connect horizontally)
                        removeWall(x, ellerCurrentRow, x + 1, ellerCurrentRow);

                        // Merge the sets
                        int oldSet = ellerSets[x + 1];
                        int newSet = ellerSets[x];

                        // Update all cells in this row that have the old set ID
                        for (int i = 0; i < width; i++) {
                            if (ellerSets[i] == oldSet) {
                                ellerSets[i] = newSet;
                            }
                        }

                        // Mark cells as visited
                        grid[ellerCurrentRow][x].visited = true;
                        grid[ellerCurrentRow][x + 1].visited = true;
                    }
                }

                // Phase 2: For each set in the current row, randomly connect at least one cell to the next row
                std::vector<int> nextRowSets(width, 0);  // Set IDs for cells in the next row

                // Initialize next row with new set IDs
                for (int x = 0; x < width; x++) {
                    nextRowSets[x] = 0;  // No set assigned yet
                }

                // For each set ID in the current row
                std::vector<bool> setConnected(ellerNextSetId, false);

                for (int x = 0; x < width; x++) {
                    int currentSet = ellerSets[x];

                    // Randomly decide if this cell connects to the next row, or must connect if no other connection yet
                    bool mustConnect = !setConnected[currentSet];
                    bool shouldConnect = mustConnect || (rng() % 3 == 0);  // 1/3 chance otherwise

                    if (shouldConnect) {
                        // Remove the bottom wall (connect vertically)
                        removeWall(x, ellerCurrentRow, x, ellerCurrentRow + 1);

                        // The cell in the next row inherits the set ID
                        nextRowSets[x] = currentSet;

                        // Mark this set as connected
                        setConnected[currentSet] = true;

                        // Mark the cells as visited
                        grid[ellerCurrentRow][x].visited = true;
                        grid[ellerCurrentRow + 1][x].visited = true;
                    }
                }

                // Prepare for the next row: assign new set IDs to cells that weren't connected
                for (int x = 0; x < width; x++) {
                    if (nextRowSets[x] == 0) {
                        nextRowSets[x] = ellerNextSetId++;
                    }
                }

                // Update to the next row
                ellerSets = nextRowSets;
                ellerCurrentRow++;
            }
            else {
                // Last row: connect all adjacent cells that are in different sets
                for (int x = 0; x < width - 1; x++) {
                    if (ellerSets[x] != ellerSets[x + 1]) {
                        // Remove the right wall
                        removeWall(x, ellerCurrentRow, x + 1, ellerCurrentRow);

                        // Mark cells as visited
                        grid[ellerCurrentRow][x].visited = true;
                        grid[ellerCurrentRow][x + 1].visited = true;

                        // Merge the sets (not strictly necessary for the last row, but for consistency)
                        int oldSet = ellerSets[x + 1];
                        int newSet = ellerSets[x];

                        for (int i = 0; i < width; i++) {
                            if (ellerSets[i] == oldSet) {
                                ellerSets[i] = newSet;
                            }
                        }
                    }
                }

                // Move to the next row (which will end the algorithm)
                ellerCurrentRow++;
            }

            madeProgress = true;
            break;
        }
        }

        return madeProgress;
    }

    // Check if there's a valid path between two adjacent cells
    bool canPass(int x1, int y1, int x2, int y2) const {
        // Check if cells are adjacent
        int dx = x2 - x1;
        int dy = y2 - y1;

        if (abs(dx) + abs(dy) != 1) {
            return false; // Cells are not adjacent
        }

        // Check if there's a wall between them
        if (dx == 1) { // Moving right
            return !grid[y1][x1].walls[1]; // Check right wall of (x1, y1)
        }
        else if (dx == -1) { // Moving left
            return !grid[y1][x1].walls[3]; // Check left wall of (x1, y1)
        }
        else if (dy == 1) { // Moving down
            return !grid[y1][x1].walls[2]; // Check bottom wall of (x1, y1)
        }
        else if (dy == -1) { // Moving up
            return !grid[y1][x1].walls[0]; // Check top wall of (x1, y1)
        }

        return false;
    }

    // Manhattan distance heuristic for A* pathfinding
    int manhattanDistance(int x1, int y1, int x2, int y2) const {
        return abs(x1 - x2) + abs(y1 - y2);
    }

    // A* pathfinding algorithm to solve the maze
    bool solveMaze(int startX, int startY, int endX, int endY) {
        // Clear previous solution
        solutionPath.clear();
        solutionFound = false;

        // Priority queue for A* algorithm: pair of (priority, position)
        // Lower priority values are processed first
        using PriorityCell = std::pair<int, std::pair<int, int>>;
        std::priority_queue<PriorityCell, std::vector<PriorityCell>, std::greater<PriorityCell>> priorityQueue;

        // Maps to store the cost and previous cell for each visited cell
        std::map<std::pair<int, int>, int> gScore; // Cost from start to current
        std::map<std::pair<int, int>, std::pair<int, int>> cameFrom; // Previous cell

        // Initialize starting point
        std::pair<int, int> start = { startX, startY };
        std::pair<int, int> end = { endX, endY };

        gScore[start] = 0;
        priorityQueue.push({ manhattanDistance(startX, startY, endX, endY), start });

        // Directions: right, down, left, up
        const int dx[] = { 1, 0, -1, 0 };
        const int dy[] = { 0, 1, 0, -1 };

        // Process cells until the queue is empty or the end is found
        while (!priorityQueue.empty()) {
            std::pair<int, int> current = priorityQueue.top().second;
            priorityQueue.pop();

            // If we reached the end, reconstruct the path
            if (current == end) {
                // Reconstruct the path
                solutionPath.clear();
                std::pair<int, int> backtracking = end;

                while (backtracking != start) {
                    solutionPath.push_back(backtracking);
                    backtracking = cameFrom[backtracking];
                }

                solutionPath.push_back(start);
                std::reverse(solutionPath.begin(), solutionPath.end());
                solutionFound = true;
                return true;
            }

            // Explore neighbors
            for (int i = 0; i < 4; i++) {
                int nx = current.first + dx[i];
                int ny = current.second + dy[i];
                std::pair<int, int> neighbor = { nx, ny };

                // Check if neighbor is valid and there's a path
                if (isValid(nx, ny) && canPass(current.first, current.second, nx, ny)) {
                    // Calculate tentative g score
                    int tentativeGScore = gScore[current] + 1;

                    // If we found a better path to the neighbor
                    if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                        // Update the best path
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = tentativeGScore;
                        int fScore = tentativeGScore + manhattanDistance(nx, ny, endX, endY);
                        priorityQueue.push({ fScore, neighbor });
                    }
                }
            }
        }

        // No path found
        return false;
    }

    // Generate the entire maze using the selected algorithm
    void generateMaze() {
        initMazeGeneration();

        while (generationInProgress) {
            performGenerationStep();
            renderMaze();
            SDL_Delay(generationSpeed);
        }
    }

    // Render the maze
    void renderMaze() {
        // Clear the screen
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        // Draw the maze cells
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int posX = x * cellSize;
                int posY = y * cellSize;

                // If visited, fill with a light color
                if (grid[y][x].visited) {
                    SDL_SetRenderDrawColor(renderer, 200, 230, 255, 255);
                    SDL_Rect cellRect = { posX + wallThickness, posY + wallThickness,
                                        cellSize - 2 * wallThickness, cellSize - 2 * wallThickness };
                    SDL_RenderFillRect(renderer, &cellRect);
                }

                // Draw walls if present
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

                if (grid[y][x].walls[0]) { // Top wall
                    SDL_Rect topWall = { posX, posY, cellSize, wallThickness };
                    SDL_RenderFillRect(renderer, &topWall);
                }
                if (grid[y][x].walls[1]) { // Right wall
                    SDL_Rect rightWall = { posX + cellSize - wallThickness, posY, wallThickness, cellSize };
                    SDL_RenderFillRect(renderer, &rightWall);
                }
                if (grid[y][x].walls[2]) { // Bottom wall
                    SDL_Rect bottomWall = { posX, posY + cellSize - wallThickness, cellSize, wallThickness };
                    SDL_RenderFillRect(renderer, &bottomWall);
                }
                if (grid[y][x].walls[3]) { // Left wall
                    SDL_Rect leftWall = { posX, posY, wallThickness, cellSize };
                    SDL_RenderFillRect(renderer, &leftWall);
                }
            }
        }

        // Draw start and end points if solver is enabled
        if (solverEnabled) {
            // Draw start point (green)
            SDL_SetRenderDrawColor(renderer, 0, 200, 0, 255);
            SDL_Rect startRect = {
                startPoint.first * cellSize + wallThickness + 2,
                startPoint.second * cellSize + wallThickness + 2,
                cellSize - 2 * wallThickness - 4,
                cellSize - 2 * wallThickness - 4
            };
            SDL_RenderFillRect(renderer, &startRect);

            // Draw end point (red)
            SDL_SetRenderDrawColor(renderer, 200, 0, 0, 255);
            SDL_Rect endRect = {
                endPoint.first * cellSize + wallThickness + 2,
                endPoint.second * cellSize + wallThickness + 2,
                cellSize - 2 * wallThickness - 4,
                cellSize - 2 * wallThickness - 4
            };
            SDL_RenderFillRect(renderer, &endRect);

            // Draw solution path if found
            if (solutionFound && !solutionPath.empty()) {
                SDL_SetRenderDrawColor(renderer, 255, 215, 0, 255); // Golden color for the path

                for (size_t i = 0; i < solutionPath.size(); i++) {
                    int pathX = solutionPath[i].first;
                    int pathY = solutionPath[i].second;

                    // Skip start and end points as they're already colored
                    if ((pathX == startPoint.first && pathY == startPoint.second) ||
                        (pathX == endPoint.first && pathY == endPoint.second)) {
                        continue;
                    }

                    SDL_Rect pathRect = {
                        pathX * cellSize + wallThickness + 5,
                        pathY * cellSize + wallThickness + 5,
                        cellSize - 2 * wallThickness - 10,
                        cellSize - 2 * wallThickness - 10
                    };
                    SDL_RenderFillRect(renderer, &pathRect);
                }
            }
        }

        // Start ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Create ImGui window
        ImGui::SetNextWindowPos(ImVec2(width * cellSize + 10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, height * cellSize - 20), ImGuiCond_FirstUseEver);
        ImGui::Begin("Maze Generator Controls", &showUI);

        // Algorithm selection
        const char* algorithmNames[] = { "Recursive Backtracking", "Prim's Algorithm", "Kruskal's Algorithm", "Eller's Algorithm" };
        static int currentAlgorithmIdx = 0;

        if (ImGui::Combo("Algorithm", &currentAlgorithmIdx, algorithmNames, IM_ARRAYSIZE(algorithmNames))) {
            currentAlgorithm = static_cast<Algorithm>(currentAlgorithmIdx);
        }

        // Generation speed slider
        ImGui::SliderInt("Generation Speed", &generationSpeed, 1, 100, "%d ms");
        ImGui::Separator();

        // Generation controls
        if (ImGui::Button("Start Generation", ImVec2(120, 30))) {
            initMazeGeneration();
        }

        ImGui::SameLine();

        if (ImGui::Button("Reset Maze", ImVec2(120, 30))) {
            resetMaze();
        }

        // Auto-generate checkbox
        ImGui::Checkbox("Auto-Generate", &autoGenerate);

        ImGui::Separator();

        // Display information about current algorithm
        ImGui::Text("Current Algorithm:");
        ImGui::Separator();

        switch (currentAlgorithm) {
        case Algorithm::RECURSIVE_BACKTRACKING:
            ImGui::TextWrapped("Recursive Backtracking uses a depth-first search with backtracking to explore and create paths.");
            break;
        case Algorithm::PRIMS:
            ImGui::TextWrapped("Prim's Algorithm starts with a single cell and expands outward, adding adjacent cells to create a minimum spanning tree.");
            break;
        case Algorithm::KRUSKALS:
            ImGui::TextWrapped("Kruskal's Algorithm connects disjoint cells, forming a minimum spanning tree by adding edges in random order.");
            break;
        case Algorithm::ELLERS:
            ImGui::TextWrapped("Eller's Algorithm processes the maze row by row, maintaining set membership to create perfect mazes with high efficiency.");
            break;
        }

        // Display generation status
        ImGui::Separator();
        ImGui::Text("Status: %s", generationInProgress ? "Generating..." : "Ready");

        // Solver section
        ImGui::Separator();
        ImGui::Text("Maze Solver");
        ImGui::Separator();

        // Enable solver checkbox
        if (ImGui::Checkbox("Enable Solver", &solverEnabled)) {
            if (solverEnabled) {
                // Default start and end points
                startPoint = { 0, 0 };
                endPoint = { width - 1, height - 1 };
            }
            else {
                // Clear solution when disabling
                solutionPath.clear();
                solutionFound = false;
            }
        }

        if (solverEnabled) {
            // Start point controls
            static int startX = 0;
            static int startY = 0;

            if (startPoint.first != startX || startPoint.second != startY) {
                startX = startPoint.first;
                startY = startPoint.second;
            }

            if (ImGui::SliderInt("Start X", &startX, 0, width - 1) ||
                ImGui::SliderInt("Start Y", &startY, 0, height - 1)) {
                startPoint = { startX, startY };
                // Clear solution when changing points
                solutionPath.clear();
                solutionFound = false;
            }

            // End point controls
            static int endX = width - 1;
            static int endY = height - 1;

            if (endPoint.first != endX || endPoint.second != endY) {
                endX = endPoint.first;
                endY = endPoint.second;
            }

            if (ImGui::SliderInt("End X", &endX, 0, width - 1) ||
                ImGui::SliderInt("End Y", &endY, 0, height - 1)) {
                endPoint = { endX, endY };
                // Clear solution when changing points
                solutionPath.clear();
                solutionFound = false;
            }

            // Solve button
            if (ImGui::Button("Solve Maze", ImVec2(120, 30))) {
                bool solved = solveMaze(startPoint.first, startPoint.second, endPoint.first, endPoint.second);
                if (!solved) {
                    ImGui::OpenPopup("No Solution");
                }
            }

            // Display solution status
            if (solutionFound) {
                ImGui::Text("Solution found! Path length: %zu", solutionPath.size());
            }
        }

        // No solution popup
        if (ImGui::BeginPopupModal("No Solution", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("No solution exists between the selected points.");
            ImGui::Text("Make sure there are no walls blocking the path.");

            if (ImGui::Button("OK", ImVec2(120, 0))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::End();

        // Render ImGui
        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());

        // Update the screen
        SDL_RenderPresent(renderer);
    }

    // Main loop
    void run() {
        SDL_Event event;
        bool quit = false;

        while (!quit) {
            // Handle events
            while (SDL_PollEvent(&event)) {
                // Pass events to ImGui
                ImGui_ImplSDL2_ProcessEvent(&event);

                if (event.type == SDL_QUIT) {
                    quit = true;
                }

                // Handle window resize
                if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    int newWidth, newHeight;
                    SDL_GetWindowSize(window, &newWidth, &newHeight);
                    // Handle resizing logic if needed
                }

                // Handle mouse clicks for selecting start/end points
                if (solverEnabled && event.type == SDL_MOUSEBUTTONDOWN) {
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        int mouseX = event.button.x;
                        int mouseY = event.button.y;

                        // Convert mouse coordinates to grid coordinates
                        int gridX = mouseX / cellSize;
                        int gridY = mouseY / cellSize;

                        // Only process if within the maze area
                        if (isValid(gridX, gridY)) {
                            if (ImGui::GetIO().KeyShift) {
                                // Set end point with Shift+Click
                                endPoint = { gridX, gridY };
                            }
                            else {
                                // Set start point with regular click
                                startPoint = { gridX, gridY };
                            }

                            // Clear solution when changing points
                            solutionPath.clear();
                            solutionFound = false;
                        }
                    }
                }
            }

            // Handle auto generation or step by step generation
            if (generationInProgress) {
                if (autoGenerate) {
                    bool madeProgress = performGenerationStep();
                    if (!madeProgress) {
                        generationInProgress = false;
                    }
                }
            }

            // Render the maze and UI
            renderMaze();

            // Cap the frame rate
            SDL_Delay(generationSpeed > 0 ? generationSpeed : 16);
        }
    }
};

int main(int argc, char* argv[]) {
    // Default maze size
    int width = 30;
    int height = 30;

    // Parse command line arguments for maze size
    if (argc == 3) {
        width = std::stoi(argv[1]);
        height = std::stoi(argv[2]);
    }

    // Create the maze generator
    MazeGenerator maze(width, height);
    std::cout << "Maze Generator started with ImGui interface" << std::endl;
    std::cout << "Use the UI to select an algorithm and control the generation" << std::endl;

    // Run the main loop
    maze.run();

    return 0;
}