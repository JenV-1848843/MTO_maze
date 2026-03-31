#ifndef CELL_H
#define CELL_H

#include "direction.h"
#include "position.h"
#include "wall.h"
#include <cstdint>

class Cell {
public:
    Cell();
    Cell(Position pos, int x = 0, int y = 0);
    ~Cell();

    Position getPosition();
    int getX();
    int getY();
    void setWall(Wall w, bool present = true);
    bool hasWall(Wall w) const;
    Cell* next = nullptr;
    bool visited = false;

private:
    Position pos;
    int x, y;
    uint8_t walls;
};

#endif