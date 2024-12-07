#include <stdio.h>
#include "solver.h"
// #include "flood.h"

#include "API.h"

#define SIMULATION_BOOL 1 // 1 if you're running the simulation, 0 if you're running the actual robot

#define CHECK_WALL_FRONT API_wallFront()
#define CHECK_WALL_LEFT API_wallLeft()
#define CHECK_WALL_RIGHT API_wallRight()

#define MOVE_FORWARD_FUNCTION API_moveForward()
#define TURN_LEFT_FUNCTION API_turnLeft()
#define TURN_RIGHT_FUNCTION API_turnRight()

#define debug(x) NULL

// choose which algorithm to use in the beginning of the run
// int algorithm;
// struct dist_maze distances;
// struct wall_maze cell_walls_info;
// struct stack update_stack;
// struct stack move_queue;

// You do not need to edit this file.
// This program just runs your solver and passes the choices
// to the simulator.

int main(int argc, char *argv[])
{
    // debug_log("Running...");
    initialize();

    // start the search
    searchRun();

    // reached the center, now calculate the shortest path
    markCenterWall();
    calculateShortestPathDistances();

    // use hand to move the mouse to the start position, and find the shortest path
    API_ackReset();
    setPosition(0, 0, NORTH);

    // run the shortest path
    fastRunWithVariableVelocity();

    while (1)
    {
        // run from the center to the start
        searchCenterToStart();

        // updateDistances();
        calculateShortestPathDistances();

        // run the shortest path
        fastRunWithVariableVelocity();
    }
}