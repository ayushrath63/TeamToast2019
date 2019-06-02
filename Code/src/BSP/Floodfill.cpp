
#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"

const unsigned short MAZE_LEN = 16;
const int INFINITE = MAZE_LEN * MAZE_LEN;


struct Cell {
    int x; 
    int y; 
};

class Floodfill : public PathFinder {
public:
    Floodfill(){initializeDistances();}
    void setMaze(const Maze* maze) {
        m_maze = maze;
    }

    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        //printDistance();
        
        // actually deciding on moves
         // move to the next smaller neighbour cell 
       if (atCenter(x, y)) {
        return Finish;
       }
       Dir heading = maze.heading;
       runFloodFill(x, y);
       unsigned short curDist = getDistance(x, y, INVALID);

       if (!maze.wallInFront() && getDistance(x, y, heading) < curDist)
       {
         return MoveForward;
       }
       else if (!maze.wallOnRight() &&
                getDistance(x, y, clockwise(heading)) < curDist)
       {
         return TurnClockwise;
       }
       else if (!maze.wallOnLeft() &&
                getDistance(x, y, counterClockwise(heading)) < curDist)
       {
         return TurnCounterClockwise;
       }
       else if (getDistance(x, y, opposite(heading)) < curDist)
       {    
         return TurnAround;
       }
       return Wait;
    }

private:
    const Maze* m_maze;
    std::stack <Cell> ffstack;
    unsigned short distances[MAZE_LEN][MAZE_LEN];
    void printDistance() {
        for (int j = MAZE_LEN-1; j>= 0 ; j--) {
            for (int i=0; i < MAZE_LEN; i ++ ) {
                std::cout << distances[i][j] << "|\t";
            }
             std::cout << std::endl;
        }
    }
    void initializeDistances()
    {
      // Distance initialization assumes middle 4 squares are finish point
      for (unsigned short y = 0; y < MAZE_LEN / 2; y++)
      {
        for (unsigned short x = 0; x < MAZE_LEN / 2; x++)
        {
          distances[y][x] = distances[y][MAZE_LEN - x - 1] =
              distances[MAZE_LEN - y - 1][x] =
                  distances[MAZE_LEN - y - 1][MAZE_LEN - x - 1] =
                      MAZE_LEN - 2 - y - x;
        }
      }

      // for (unsigned short y = 0; y < MAZE_LEN; y++)
      // {
      //   for (unsigned short x = 0; x < MAZE_LEN; x++)
      //   {
      //     runFloodFill(x, y);
      //   }
      // }
    }

    void stack_push(int x, int y) {
     Cell c = {x,y};
     ffstack.push(c);
    }


    unsigned short getDistance(unsigned x, unsigned y, Dir heading) {
     if (x >= MAZE_LEN || y >= MAZE_LEN)
        return INFINITE;
     switch (heading) {
     case NORTH:
         return y + 1 < MAZE_LEN ? distances[x][y+1] : INFINITE;
     case SOUTH:
         return y > 0 ? distances[x][y-1] : INFINITE;
     case EAST:
         return x + 1 < MAZE_LEN ? distances[x + 1][y] : INFINITE;
     case WEST:
         return x > 0 ? distances[x - 1][y] : INFINITE;
     case INVALID:
         return distances[x][y];
     default:
         return INFINITE;
     }
    } 

    /**
     * @brief Find the minimum distance of the cells adjacent to the one at (x, y).
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     *
     * @retval  The minimum distance value of the neighboring cells.
     */
    unsigned short findMinDistanceOfNeighbors(unsigned short x, unsigned short y)
    {
      unsigned short minDist = INFINITE;
      unsigned short northDist = getDistance(x, y, NORTH);
      unsigned short eastDist = getDistance(x, y, EAST);
      unsigned short southDist = getDistance(x, y, SOUTH);
      unsigned short westDist = getDistance(x, y, WEST);
      if (m_maze->isOpen(x, y, NORTH) && northDist < minDist)
        minDist = northDist;
      if (m_maze->isOpen(x, y, EAST) && eastDist < minDist)
        minDist = eastDist;
      if (m_maze->isOpen(x, y, SOUTH) && southDist < minDist)
        minDist = southDist;
      if (m_maze->isOpen(x, y, WEST) && westDist < minDist)
        minDist = westDist;
      return minDist;
    }

    void setDistance(unsigned short x, unsigned short y, unsigned short val){
        //std::cout << "set distance at x: " << x << " y: " << y << " dis: " << val  << std::endl;

        distances[x][y] = val;
    }
    void runFloodFill(int x, int y) {

     //push nerighbour cells onto stack 
        stack_push(x,y);
      if (!m_maze->isOpen(x, y, NORTH))
        stack_push(x, y + 1);
      if (!m_maze -> isOpen(x, y, EAST))
        stack_push(x + 1, y);
      if (!m_maze -> isOpen(x, y, SOUTH))
        stack_push(x, y - 1);
      if (!m_maze -> isOpen(x, y, WEST))
        stack_push(x - 1, y);

      while (!ffstack.empty())
      {
        Cell cur = ffstack.top();
        ffstack.pop();
        //std::cout << "stack top. x: " <<  cur.x << " y:" << cur.y << std::endl;
        // if already in the center 
        if (getDistance(cur.x, cur.y, INVALID) == 0)
          continue;
        unsigned short curDist = getDistance(cur.x, cur.y, INVALID);
        unsigned short minDist = findMinDistanceOfNeighbors(cur.x, cur.y);
        // if (curDist == minDist + 1 || minDist >= INFINITE)
        //   continue;
        //std::cout << "cur distance: " <<  curDist << " minimumDist" << minDist << std::endl;
        if (curDist > minDist || minDist >= INFINITE)
          continue;
        setDistance(cur.x, cur.y, minDist + 1);
        if (m_maze->isOpen(cur.x, cur.y, NORTH))
          stack_push(cur.x, cur.y + 1);
        if (m_maze->isOpen(cur.x, cur.y, EAST))
          stack_push(cur.x + 1, cur.y);
        if (m_maze->isOpen(cur.x, cur.y, SOUTH))
          stack_push(cur.x, cur.y - 1);
        if (m_maze->isOpen(cur.x, cur.y, WEST))
          stack_push(cur.x - 1, cur.y);
      }
      //std::cout << "return from floodfill" << std::endl; 
    }

    bool atCenter(unsigned short x, unsigned short y)
    {
      unsigned midpoint = MAZE_LEN / 2;

      return (x == midpoint && y == midpoint) ||
             (x == midpoint - 1 && y == midpoint) ||
             (x == midpoint && y == midpoint - 1) ||
             (x == midpoint - 1 && y == midpoint - 1);
    }


};