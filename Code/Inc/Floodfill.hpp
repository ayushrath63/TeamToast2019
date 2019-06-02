#ifndef __FLOODFILL_HPP__
#define __FLOODFILL_HPP__

#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"
#include <etl/stack.h>
const unsigned short MAZE_LEN = 16;
const int INFINITE = MAZE_LEN * MAZE_LEN;


struct Cell {
    int x; 
    int y; 
};



class Floodfill : public PathFinder {
public:
    Floodfill();
    void setMaze(const Maze* maze);
    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze);

private:
    const Maze* m_maze;
    etl::stack <Cell,256> ffstack;
    unsigned short distances[MAZE_LEN][MAZE_LEN];
    void printDistance();
    void initializeDistances();
    unsigned short getDistance(unsigned x, unsigned y, Dir heading);
    unsigned short findMinDistanceOfNeighbors(unsigned short x, unsigned short y);
    void setDistance(unsigned short x, unsigned short y, unsigned short val);
    void runFloodFill(int x, int y);
    bool atCenter(unsigned short x, unsigned short y);
    void stack_push(int x, int y);

};

#endif