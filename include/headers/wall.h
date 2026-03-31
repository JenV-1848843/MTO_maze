#ifndef WALL_H 
#define WALL_H

#include <cstdint>

// Enum to be able to store walls of cells in a single uint8_t later
enum Wall : uint8_t {
    /*
        << is the bit shift operator
        here the 1 after the = represents the uint8_t of the number 1 (00000001)
        x << n will shift the bits of x n places to the left and append zeros to the right (no bit wrapping)
    */

    TOP    = 1 << 0, // (00000001) = 1
    RIGHT  = 1 << 1, // (00000010) = 2
    BOTTOM = 1 << 2, // (00000100) = 4
    LEFT   = 1 << 3  // (00001000) = 8
};

#endif