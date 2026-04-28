#ifndef DIRECTION_H
#define DIRECTION_H

#include <string>

enum class Direction {
    UP,
    DOWN,
    LEFT,
    RIGHT
};

inline std::string to_string(Direction dir) {
    switch (dir) {
        case Direction::UP: return "UP";
        case Direction::DOWN: return "DOWN";
        case Direction::LEFT: return "LEFT";
        case Direction::RIGHT: return "RIGHT";
        default: return "UNKNOWN";
    }
};

#endif