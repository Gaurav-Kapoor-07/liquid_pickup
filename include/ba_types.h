#ifndef BA_TYPES_H
#define BA_TYPES_H

#pragma region includes

#include <cmath>

#pragma endregion

/**
 * @brief Struct to store coordinates
 * 
 */
struct Coordinates{
    double x;
    double y;
    double z;
    double r;
    bool is_picked;
    inline bool operator==(const Coordinates t1){
        return (pow(x-t1.x,2)+pow(y-t1.y,2)+pow(z-t1.z,2))<std::max(r, t1.r)+0.01;
    };
} ;

// /**
//  * @brief Struct to specify single locations of the path where the robot has to drive.
//  * 
//  */
// struct PathLocation {
//     double x;
//     double y;
//     double orientation;
// };

// /**
//  * @brief Enum to specify whether to push/pop in front or back of queue.
//  * 
//  */
// enum QueueSide {
//     front,
//     back
// };

#endif