#ifndef SOLVER_H
#define SOLVER_H

#include "queue.h"

#include <AML_GlobalVariable.h>

#include <AML_Buzzer.h>
#include <AML_LedDebug.h>
#include <AML_Switch.h>

#include <AML_IMU.h>
#include <AML_IRSensor.h>
#include <AML_Encoder.h>

#include <AML_MotorControl.h>

typedef enum Heading
{
    NORTH,
    EAST,
    SOUTH,
    WEST
} Heading;
typedef enum Action
{
    LEFT,
    FORWARD,
    RIGHT,
    IDLE
} Action;

/* MAZE CONSTANTS */
#define MAZE_SIZE 16
/*
Sets the different types of cells (the walls around a given cell) in the format:
    _TopWall RightWall BottomWall LeftWall
    1: there is a wall
    0: there isn't a wall
*/
#define _0000 0
#define _0001 1
#define _0010 2
#define _0011 3
#define _0100 4
#define _0101 5
#define _0110 6
#define _0111 7
#define _1000 8
#define _1001 9
#define _1010 10
#define _1011 11
#define _1100 12
#define _1101 13
#define _1110 14
#define _1111 15 // not actually possible in a maze
/*
_1000 cho tường phía Bắc
_0100 cho tường phía Đông
_0010 cho tường phía Nam
_0001 cho tường phía Tây
*/
extern unsigned int maze[MAZE_SIZE][MAZE_SIZE];
extern int distances[MAZE_SIZE][MAZE_SIZE];

struct Coordinate
{
    int x;
    int y;
};

void initialize(void);
void updateMaze(void);      // updates the maze array with the walls around the mouse's current position
void updateDistances(void); // the "floodfill" algorithm
void resetDistances(void);
int xyToSquare(int x, int y);
struct Coordinate squareToCoord(int square);
int isWallInDirection(int x, int y, Heading direction);
void updateHeading(Action nextAction);
void updatePosition(Action nextAction);
Action solver(void);
Action leftWallFollower(void);
Action floodFill(void);

#endif
