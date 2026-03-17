#ifndef MAZE_H
#define MAZE_H

#include <array>
#include "cell.h"
#include "position.h"
#include "config.h"

class Maze {
    public:
        Maze();
        void print();
        std::array<std::array<Cell, amountCellRows>, amountCellCols>& getConfig();
        void bfs(Cell& dest);
        // ~Maze();

    private:
        std::array<std::array<Cell, amountCellRows>, amountCellCols> config;
        void setOuterWalls(std::array<std::array<Cell, amountCellRows>, amountCellCols>& config);
};

#endif