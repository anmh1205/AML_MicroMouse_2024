#include <stdio.h>
#include "solver.h"
#include "API.h"

#define SIMULATION_BOOL 1 // 1 if you're running the simulation, 0 if you're running the actual robot

#define CHECK_WALL_FRONT API_wallFront()
#define CHECK_WALL_LEFT API_wallLeft()
#define CHECK_WALL_RIGHT API_wallRight()

#define MOVE_FORWARD_FUNCTION API_moveForward()
#define TURN_LEFT_FUNCTION API_turnLeft()
#define TURN_RIGHT_FUNCTION API_turnRight()

// You do not need to edit this file.
// This program just runs your solver and passes the choices
// to the simulator.
int main(int argc, char *argv[])
{
    // debug_log("Running...");
    initialize();
    while (1)
    {
        Action nextMove = solver();

        // displays distances on the squares in the simulator
        for (int x = 0; x < MAZE_SIZE; ++x)
        {
            for (int y = 0; y < MAZE_SIZE; ++y)
            {
#if SIMULATION_BOOL
                API_setText(x, y, distances[x][y]);
#endif
            }
        }

        switch (nextMove)
        {
        case FORWARD:
            // API_moveForward();
            MOVE_FORWARD_FUNCTION;
            break;
        case LEFT:
            // API_turnLeft();
            TURN_LEFT_FUNCTION;
            break;
        case RIGHT:
            // API_turnRight();
            TURN_RIGHT_FUNCTION;
            break;
        case IDLE:
            break;
        }
    }
}