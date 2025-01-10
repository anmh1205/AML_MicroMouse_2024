#ifndef SOLVER_H
#define SOLVER_H

#include <stdint.h>

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

// extern unsigned int maze[MAZE_SIZE][MAZE_SIZE];
extern int16_t distances[MAZE_SIZE][MAZE_SIZE];
extern uint8_t wall_maze[MAZE_SIZE][MAZE_SIZE][4];
// extern int wall_maze[MAZE_SIZE][MAZE_SIZE][4] = {0};

struct Coordinate
{
    int x;
    int y;
};

void setPriorityHeading(int direction);
void searchRun(void);
void searchCenterToStart(void);
void firstFastRun(void);
void loopRun(void);
void setPosition(int x, int y, int direction);
void markCenterWall(void);
void initialize(void);
void updateMaze(void);      // updates the maze array with the walls around the mouse's current position
void updateDistances(void); // the "floodfill" algorithm
void calculateShortestPathDistances(void);
void fastRunWithVariableVelocity(void);
void resetDistances(void);
int xyToSquare(int x, int y);
struct Coordinate squareToCoord(int square);
int isWallInDirection(int x, int y, Heading direction);
void updateHeading(Action nextAction);
void updatePosition(Action nextAction);
int getReachingCenter(void);
Action solver(void);
Action floodFill(void);

#endif