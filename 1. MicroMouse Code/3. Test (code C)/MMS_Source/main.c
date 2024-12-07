#include <stdio.h>
#include "solver.h"
#include "API.h"


// You do not need to edit this file.
// This program just runs your solver and passes the choices
// to the simulator.
int main(int argc, char* argv[]) {
    debug_log("Running...");
    initialize();
    while (1) {
        Action nextMove = solver();

        // displays distances on the squares in the simulator
        for (int x = 0; x < MAZE_SIZE; ++x) {
            for (int y = 0; y < MAZE_SIZE; ++y) {
                API_setText(x, y, distances[x][y]);
            }
        }
        
        switch(nextMove){
            case FORWARD:
                API_moveForward();
                break;
            case LEFT:
                API_turnLeft();
                break;
            case RIGHT:
                API_turnRight();
                break;
            case IDLE:
                break;
        }
    }
}