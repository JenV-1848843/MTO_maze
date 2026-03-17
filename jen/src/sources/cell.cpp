#include "../headers/cell.h"
#include "../headers/wall.h"

Cell::Cell() : pos(Position::A1), x(0), y(0), walls(0) {}

Cell::Cell(Position position, int x, int y) : pos(position), x(x), y(y), walls(0) {}

Cell::~Cell() {}

void Cell::setWall(Wall w, bool present) {
    if (present)
        walls |= w;
    else
        walls &= ~w;
}

bool Cell::hasWall(Wall w) const {
    return walls & w;
}

Position Cell::getPosition() { return pos; }
int Cell::getX() { return x; }
int Cell::getY() { return y; }