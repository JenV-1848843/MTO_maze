#include "../headers/maze.h"
#include "../headers/wall.h"
#include "../headers/position.h"

#include <iostream>
#include <cstddef>
#include <queue>

Maze::Maze(){
    for (int x = 0; x < amountCellCols; ++x)
    {
        for (int y = 0; y < amountCellRows; ++y)
        {
            this->config[x][y] = Cell(static_cast<Position>(x*y + y), x, y);
        }
    }

    this->setOuterWalls(this->config);
}

void Maze::print() {
    // For each row
    for (int y = 0; y < amountCellRows; ++y) {

        // Print the top walls of this row
        for (int x = 0; x < amountCellCols; ++x) {
            const Cell& c = config[x][y];

            // corner
            std::cout << "+";

            // horizontal wall
            if (c.hasWall(Wall::TOP))
                std::cout << "---";
            else
                std::cout << "   ";
        }
        std::cout << "+\n"; // rightmost corner

        // Print the cell contents + vertical walls
        for (int x = 0; x < amountCellCols; ++x) {
            const Cell& c = config[x][y];

            // left wall
            if (c.hasWall(Wall::LEFT))
                std::cout << "|";
            else
                std::cout << " ";

            // cell interior (just empty space for now)
            std::cout << "   ";
        }

        // rightmost wall for the row
        if (config[amountCellCols - 1][y].hasWall(Wall::RIGHT))
            std::cout << "|\n";
        else
            std::cout << " \n";
    }

    // Print the bottom walls of the last row
    for (int x = 0; x < amountCellCols; ++x) {
        const Cell& c = config[x][amountCellRows - 1];
        std::cout << "+";
        if (c.hasWall(Wall::BOTTOM))
            std::cout << "---";
        else
            std::cout << "   ";
    }
    std::cout << "+\n";
};

void Maze::setOuterWalls(std::array<std::array<Cell, amountCellRows>, amountCellCols>& grid)
{
    for (size_t x = 0; x < amountCellCols; ++x)
    {
        for (size_t y = 0; y < amountCellRows; ++y)
        {
            // Top row
            if (y == 0)
                grid[x][y].setWall(Wall::TOP, true);

            // Bottom row
            if (y == amountCellRows - 1)
                grid[x][y].setWall(Wall::BOTTOM, true);

            // Left column
            if (x == 0)
                grid[x][y].setWall(Wall::LEFT, true);

            // Right column
            if (x == amountCellCols - 1)
                grid[x][y].setWall(Wall::RIGHT, true);
        }
    }
};

std::array<std::array<Cell, amountCellRows>, amountCellCols>& Maze::getConfig() {
    return this->config;
};

void Maze::bfs(Cell& destination) {
    // Reset all next pointers and visited flags before running
    for (int x = 0; x < amountCellCols; ++x)
        for (int y = 0; y < amountCellRows; ++y) {
            config[x][y].next = nullptr;
            config[x][y].visited = false;
        }

    std::queue<Cell*> queue;
    Cell* dest = &config[destination.getX()][destination.getY()];

    // Mark destination as visited before pushing
    dest->visited = true;
    queue.push(dest);

    while (!queue.empty()) {
        Cell* current = queue.front();
        queue.pop();

        int cx = current->getX();
        int cy = current->getY();

        struct Neighbour { int nx, ny; Wall wallFromNeighbour; };
        Neighbour neighbours[] = {
            { cx,     cy - 1, Wall::BOTTOM },
            { cx,     cy + 1, Wall::TOP    },
            { cx - 1, cy,     Wall::RIGHT  },
            { cx + 1, cy,     Wall::LEFT   },
        };

        for (int i = 0; i < 4; ++i) {
            int nx = neighbours[i].nx;
            int ny = neighbours[i].ny;
            Wall wallFromNeighbour = neighbours[i].wallFromNeighbour;

            if (nx < 0 || nx >= amountCellCols || ny < 0 || ny >= amountCellRows)
                continue;

            Cell* neighbour = &config[nx][ny];

            if (neighbour->visited)
                continue;

            if (neighbour->hasWall(wallFromNeighbour))
                continue;

            neighbour->visited = true;
            neighbour->next = current;
            queue.push(neighbour);
        }
    }
}

void Maze::resetWalls() {
    for (size_t x = 0; x < amountCellCols; ++x) {
        for (size_t y = 0; y < amountCellRows; ++y) {
            config[x][y].setWall(Wall::TOP, false);
            config[x][y].setWall(Wall::BOTTOM, false);
            config[x][y].setWall(Wall::LEFT, false);
            config[x][y].setWall(Wall::RIGHT, false);
        }
    }

    // Restore boundary walls
    this->setOuterWalls(this->config);
}

Direction Maze::directionTo(Cell* from, Cell* to) {
    int dx = to->getX() - from->getX();
    int dy = to->getY() - from->getY();
    if      (dx ==  1) return Direction::RIGHT;
    else if (dx == -1) return Direction::LEFT;
    else if (dy ==  1) return Direction::DOWN;
    else               return Direction::UP;
}

int Maze::stepsUntilTurn(Cell& from) {
    Cell* current = &config[from.getX()][from.getY()];

    // No path found from this cell
    if (current->next == nullptr)
        return 0;

    Direction initialDir = directionTo(current, current->next);
    int steps = 0;

    while (current->next != nullptr) {
        Direction dir = directionTo(current, current->next);
        if (dir != initialDir)
            break;
        ++steps;
        current = current->next;
    }

    return steps;
}

double gridToWorld(int coord) {
    double cellWidth = (double)outerWallLength/8;
    return (coord * cellWidth) + cellWidth/2;
};

double Maze::mmsUntilTurn(Cell& from, BallPosition* pos) {
    Cell* current = &config[from.getX()][from.getY()];

    // No path found from this cell
    if (current->next == nullptr)
        return 0.0;

    Direction initialDir = directionTo(current, current->next);
    int steps = 0;
    Cell* dest = current->next;

    while (current->next != nullptr) {
        Direction dir = directionTo(current, current->next);
        if (dir != initialDir)
            break;
        ++steps;
        dest = current->next;
        current = current->next;
    }

    double fromMm = 0.0;
    double destMm = 0.0;

    if (initialDir == Direction::RIGHT) {
        double fromMm = pos->mmX;
        double destMm = gridToWorld(dest->getX());
        return destMm - fromMm;
    } else if (initialDir == Direction::LEFT) {
        double fromMm = pos->mmX;
        double destMm = gridToWorld(dest->getX());
        return fromMm - destMm;
    } else if (initialDir == Direction::DOWN) {
        double fromMm = pos->mmY;
        double destMm = gridToWorld(dest->getY());
        return destMm - fromMm;
    } else if (initialDir == Direction::UP) {
        double fromMm = pos->mmY;
        double destMm = gridToWorld(dest->getY());
        return fromMm - destMm;
    } else {
        return 0.0;
    }
}

// Maze::~Maze(){}
