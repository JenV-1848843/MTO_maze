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

// Maze::~Maze(){}