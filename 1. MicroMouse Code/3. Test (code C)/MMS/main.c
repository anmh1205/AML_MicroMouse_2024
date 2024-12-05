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
    debug_log("Starting search");
    searchRun();

    // reached the center, now calculate the shortest path
    debug_log("Reached center");
    calculateFirstShortestPathDistances();

    API_ackReset();

    // run the shortest path
    debug_log("Running shortest path");
    // firstFastRun();
    fastRunWithVariableVelocity();

    // use hand to move the mouse to the start position, and find the shortest path

    while (1)
    {
        // if (getReachingCenter())
        // {
        //     // debug_log("Reaching center");
        //     fastRun();
        // }
        // else
        // {
        //     // debug_log("Searching");
        //     searchRun();
        // }

        // RUN_MODE_1();
    }
}