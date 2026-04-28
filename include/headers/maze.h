#ifndef MAZE_H
#define MAZE_H

#include <array>
#include "cell.h"
#include "position.h"
#include "config.h"
#include "ballPosition.h"
#include "direction.h"

class Maze {
    public:
        Maze();
        void print();
        std::array<std::array<Cell, amountCellRows>, amountCellCols>& getConfig();
        void bfs(Cell& dest);
        void resetWalls();
        int stepsUntilTurn(Cell& from);
        Direction directionTo(Cell* from, Cell* to);
        double mmsUntilTurn(Cell& from, BallPosition* pos);
        // ~Maze();

    private:
        std::array<std::array<Cell, amountCellRows>, amountCellCols> config;
        void setOuterWalls(std::array<std::array<Cell, amountCellRows>, amountCellCols>& config);
};

#endif
