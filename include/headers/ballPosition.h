#include <cstdint>
#include "position.h"

#ifndef BALLPOSITION_H
#define BALLPOSITION_H

struct BallPosition {
    uint64_t id;
    int x, y;
    double pixelX, pixelY;
    double mmX, mmY;
    bool found = false;
    // Position pos;
};

#endif