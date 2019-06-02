#ifndef __FLOODFILL_H__
#define __FLOODFILL_H__

#include "main.h"
#include <etl/stack.h>

#define MAZE_LEN 16;

class Cell {
public:
	Cell (int x, int y); 
	//
	update(int north, int south, int west, int south);
private:
	int x; 
	int y; 
	int north; 
	int south; 
	int west; 
	int east; 
};

enum struct Direction:uint8_t{
	NORTH,
	SOUTH,
	WEST,
	EAST
};

// updates all the distance in distance Maze
// pass in current cell. update untill all cell met condition  
void runfloodfill(Cell cur);
void forward() {
	
}

class Floodfill {
public: 
	void updateMaze()
	void nextMove()

private: 
	etl::stack<Cell, 256> sk;
	int distanceMaze[MAZE_LEN][MAZE_LEN];
	int wallMaze[MAZE_LEN][MAZE_LEN]
	void initDistanceMaze(); // initialize the maze based on manhattan distance
	void initWallMaze(); 
	Cell curCell;
	Cell goal; 
	Direction curDir

	

}


#endif