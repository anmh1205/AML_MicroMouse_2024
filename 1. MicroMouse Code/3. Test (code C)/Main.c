// #include <API.h>
// #include <AML_Maze.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define SIZE 16 // Size of one dimention of Map

// Directions
#define NORTH 0 // Bac
#define EAST 1  // Dong
#define SOUTH 2 // Nam
#define WEST 3  // Tay

// Shortcut Constants
#define MAPIJ this_maze->map[i][j]
#define MAP this_maze->map
#define FLOODVAL this_node->floodval
#define ROW this_node->row
#define COL this_node->column
#define VISITED this_node->visited
#define LEFT this_node->left
#define RIGHT this_node->right
#define UP this_node->up
#define DOWN this_node->down

// Stack Constants
#define SPI 1 // Stack Pointer Index
#define SSI 0 // Stack Size Index
#define STACK_OFFSET 2
#define STACKSIZE 80

// Solver Constants - will be used on mouse
#define START_X 15
#define START_Y 0
#define LARGEVAL 301

// Solver Constants - for command line simulation only
#define NEWLINE 13
#define YES 'y'
#define NO 'n'

/* Main template constants */
#define ONECELL 61
#define LEFT_WALL_SENSED 50
#define FRONT_WALL_SENSED 400
#define RIGHT_WALL_SENSED 50
#define LEFT_BASE_SPEED 23000
#define RIGHT_BASE_SPEED 23000
#define P_VAL 7
#define D_VAL 0
#define TURN_LEFT_COUNT 17
#define TURN_RIGHT_COUNT 18
#define ABOUT_FACE_COUNT 39
#define CENTER 2000
#define LEFT_TWO_AWAY 570
#define RIGHT_TWO_AWAY 630

int debug_on = 0;

typedef struct Node
{

    /* data fields */
    short floodval;
    short row;
    short column;
    short visited;

    /* pointers to neighbors */
    struct Node *left;
    struct Node *right;
    struct Node *up;
    struct Node *down;

} Node;

typedef struct Maze
{

    Node *map[SIZE][SIZE];

} Maze;

typedef struct Stack
{

    short properties[STACK_OFFSET];
    Node *the_stack[STACKSIZE];

} Stack;



FILE *fp;




Node *new_Node(const short i, const short j)
{

    Node *this_node;
    short halfsize;

    if (debug_on)
        printf("allocating %hd, %hd\n", i, j);

    this_node = (Node *)malloc(sizeof(Node));
    halfsize = SIZE / 2;

    ROW = i;
    COL = j;
    VISITED = FALSE;

    /* Initializing the flood value at this coord
         NOTE : Right now this only works when SIZE is even -- which is ok */
    if (i < halfsize && j < halfsize)
        FLOODVAL = (halfsize - 1 - i) + (halfsize - 1 - j);

    else if (i < halfsize && j >= halfsize)
        FLOODVAL = (halfsize - 1 - i) + (j - halfsize);

    else if (i >= halfsize && j < halfsize)
        FLOODVAL = (i - halfsize) + (halfsize - 1 - j);

    else
        FLOODVAL = (i - halfsize) + (j - halfsize);

    return this_node;
}

/* Maze Constructor */
Maze *new_Maze()
{

    Maze *this_maze;
    short i, j;

    this_maze = (Maze *)malloc(sizeof(Maze));

    /* Allocate a new Node for each coord of maze */
    for (i = 0; i < SIZE; ++i)
        for (j = 0; j < SIZE; ++j)
            MAPIJ = new_Node(i, j);

    /* setting the neighbors ptrs... must be done after all cells allocated */
    for (i = 0; i < SIZE; i++)
        for (j = 0; j < SIZE; j++)
        {
            MAPIJ->left = (j == 0) ? NULL : (this_maze->map[i][j - 1]);
            MAPIJ->right = (j == SIZE - 1) ? NULL : (this_maze->map[i][j + 1]);
            MAPIJ->up = (i == 0) ? NULL : (this_maze->map[i - 1][j]);
            MAPIJ->down = (i == SIZE - 1) ? NULL : (this_maze->map[i + 1][j]);
        }

    return this_maze;
}

/* Node Destructor */
void delete_Node(Node **npp)
{

    /* debug statements */
    if (debug_on)
        printf("deallocating %d, %d\n", (*npp)->row, (*npp)->column);

    free(*npp);
    *npp = 0;
}

/* Maze Destructor */
void delete_Maze(Maze **mpp)
{

    short i, j;

    for (i = 0; i < SIZE; i++)
        for (j = 0; j < SIZE; j++)
            delete_Node(&((*mpp)->map[i][j]));

    free(*mpp);
    *mpp = 0;
}

int isWall(Maze *this_maze, int i, int j, int direction)
{
    switch (direction)
    {
    case NORTH:
        return (MAPIJ->up == NULL) ? TRUE : FALSE;
    case EAST:
        return (MAPIJ->right == NULL) ? TRUE : FALSE;
    case SOUTH:
        return (MAPIJ->down == NULL) ? TRUE : FALSE;
    case WEST:
        return (MAPIJ->left == NULL) ? TRUE : FALSE;
    default:
        return FALSE;
    }
}

void print_map(Maze *this_maze, int x, int y, int direction)
{
    int i, j;

    // vẽ tường ở đỉnh trên (hướng bắc bản đồ)
    for (j = 0; j < SIZE; ++j)
    {
        printf("+ --- ");
    }
    printf("+\n");

    // viết số đánh dấu và vẽ tường cho từng hàng (từ bắc xuống nam)
    for (i = 0; i < SIZE; ++i)
    {
        printf("|"); // mép hướng tây luôn là tường

        // for để vẽ từ đầu hàng đến cuối hàng (tây sang đông)
        for (j = 0; j < SIZE; ++j)
        {
            /*
                trong for này dùng if để căn chỉnh vị trí cho trường hợp giá trị floodfill có một chữ số và hai chữ số
                ví dụ:
                    11 10 9 8 7
                    10 9 8 7

                nếu không dùng căn chỉnh mà cứ ghi bằng 1 hàm printf với số kí tự space như nhau thì từ cột thứ 2 trên ví dụ sẽ bị lệch hàng
            */
            if (MAPIJ->floodval >= 10)
            {
                if (i == x && j == y)
                {
                    printf(" %d", MAPIJ->floodval);
                    switch (direction)
                    {
                    case NORTH:
                        printf("^");
                        break;
                    case EAST:
                        printf(">");
                        break;
                    case WEST:
                        printf("<");
                        break;
                    case SOUTH:
                        printf("v");
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    printf("  %d", MAPIJ->floodval);
                }
            }
            else
            {
                if (i == x && j == y)
                {
                    printf(" %d ", MAPIJ->floodval);
                    switch (direction)
                    {
                    case NORTH:
                        printf("^");
                        break;
                    case EAST:
                        printf(">");
                        break;
                    case WEST:
                        printf("<");
                        break;
                    case SOUTH:
                        printf("v");
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    printf("  %d ", MAPIJ->floodval);
                }
            }


            // (đọc hàm isWall) check nếu có tường ở hướng tây thì sẽ ghi kí tự | để biểu diễn tường, không có thì ghi kí tự space để căn chỉnh cho thẳng cột
            if (isWall(this_maze, i, j, EAST))
            {
                printf(" |");
            }
            else
            {
                printf("  ");
            }
        }
        printf("\n"); // hết 1 hàng, xuống dòng để ghi dòng tiếp theo

        /*
            đoạn này để ghi hàng phụ trợ giữa các hàng giá trị, để biểu diễn tường theo phương ngang
            ví dụ:
            1      |  9    8    7    6    5    4    3    2    2    3    4    5    6    7    8    9 |
            2      +    +    +    +    +    +    +    +    +    +    +    +    +    +    +    +    +
            3      |  8    7    6    5    4    3    2    1    1    2    3    4    5    6    7    8 |

            thì đoạn này để ghi những hàng như hàng thứ 2.
        */
        for (j = 0; j < SIZE; ++j) // từ tây sang đông
        {
            printf("+");             // điểm mạng ở đầu hàng
            if (MAPIJ->down == NULL) // trường hợp con trỏ của node hàng 1 (như ví dụ trên) không trỏ đến gì, nghĩa là bị tường ngăn cách ở bên dưới (hướng nam) thì ghi -- biểu thị tường ngăn cách
            {
                printf(" --- ");
            }
            else // không có tường thì ghi kí tự space để căn chỉnh
            {
                printf("     ");
            }
        }
        printf("+\n"); // ghi kí tự điểm mạng ở cuối và xuống hàng
    }
}

int main(int argc, char *argv[])
{
    // fp = fopen("input.txt", "r");
    // if (fp == NULL)
    // {
    //     printf("Error opening file\n");
    //     return 0;
    // }



    Maze *myMaze = new_Maze();
    myMaze->map[0][0]->down = NULL;

    print_map(myMaze, 0, 0, EAST);
}
