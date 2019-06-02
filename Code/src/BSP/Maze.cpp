#include <iostream>
#include "IRSensor.hpp"
#include "Maze.h"


#define ARRAY_SIZE(a) (sizeof(a)/sizeof(*a))


Maze::Maze(PathFinder *pathFinder)
: heading(NORTH), pathFinder(pathFinder), mouseX(0), mouseY(0) {
   
    wallNS.setAll();
    wallEW.setAll();

    for (int i = 0; i < MAZE_LEN; i++)
      {
        setWall(i, 0, SOUTH);
        setWall(0, i, WEST);
        setWall(i, MAZE_LEN - 1, NORTH);
        setWall(MAZE_LEN - 1, i, EAST);
      }
    // Encoding stores wall/no wall in WSEN in the least significant bits
    // Data is stored in column major order
    // const unsigned westMask  = 1 << 3;
    // const unsigned southMask = 1 << 2;
    // const unsigned eastMask  = 1 << 1;
    // const unsigned northMask = 1 << 0;

    // for(unsigned col = 0; col < MazeDefinitions::MAZE_LEN; col++) {
    //     for(unsigned row = 0; row < MazeDefinitions::MAZE_LEN; row++) {
    //         const unsigned char cell = MazeDefinitions::mazes[mazeIndex][col][row];

    //         if((cell & northMask) == 0 && row != MazeDefinitions::MAZE_LEN) {
    //             setOpen(col, row, NORTH);
    //         }

    //         if((cell & southMask) == 0 && row != 0) {
    //             setOpen(col, row, SOUTH);
    //         }

    //         if((cell & westMask) == 0 && col != 0) {
    //             setOpen(col, row, WEST);
    //         }

    //         if((cell & eastMask) == 0 && col != MazeDefinitions::MAZE_LEN) {
    //             setOpen(col, row, EAST);
    //         }
    //     }
    // }
}
void Maze::printMaze(void)
{
  int n;
char printbuf[128];
sprintf(printbuf,"Print Maze \r\n");
print((uint8_t*)printbuf);
  for (int y = MAZE_LEN - 1; y >= 0; y--)
  {
    for (int x = 0; x < MAZE_LEN; x++)
    {
      n = !isOpen(x, y, NORTH) + !isOpen(x, y, EAST) * 2 + !isOpen(x, y, SOUTH) * 4 + !isOpen(x, y, WEST) * 8;
      // n = isOpen(x, y, NORTH);
    sprintf(printbuf,"%2d", n);
    print((uint8_t*)printbuf);

    }
    sprintf(printbuf,"\r\n");
    print((uint8_t*)printbuf);
  }
}
bool Maze::isOpen(unsigned x, unsigned y, Dir d) const {
    switch(d) {
        case NORTH:
            return wallNS.get(x, y+1);
        case SOUTH:
            return wallNS.get(x, y);
        case EAST:
            return wallEW.get(x+1, y);
        case WEST:
            return wallEW.get(x, y);
        case INVALID:
        default:
            return false;
    }
}

void Maze::setOpen(unsigned x, unsigned y, Dir d) {
    switch(d) {
        case NORTH:
            return wallNS.set(x, y+1);
        case SOUTH:
            return wallNS.set(x, y);
        case EAST:
            return wallEW.set(x+1, y);
        case WEST:
            return wallEW.set(x, y);
        case INVALID:
        default:
            return;
    }
}
void Maze::setWall(unsigned x, unsigned y, Dir d) { // clear a wall 
    switch(d) {
        case NORTH:
            return wallNS.clear(x, y+1);
        case SOUTH:
            return wallNS.clear(x, y);
        case EAST:
            return wallEW.clear(x+1, y);
        case WEST:
            return wallEW.clear(x, y);
        case INVALID:
        default:
            return;
    }
}

void Maze::discoverWalls() {
    // set wall when find walls 

    unsigned x = mouseX;
    unsigned y = mouseY;
    switch (heading) {
        case NORTH:
            if (ifdetectedFrontWall()) {
                 setWall(x,y,NORTH);
            } else {
                setOpen (x,y,NORTH);
            }
            if (ifdetectedLeftWall()) {
                 setWall(x,y,WEST);
            } else {
                setOpen (x,y,WEST);
            }
            if (ifdetectedRightWall()) {
                 setWall(x,y,EAST);
            } else {
                setOpen (x,y,EAST);
            }
            break;
        case SOUTH:
            if (ifdetectedFrontWall()) {
                 setWall(x,y,SOUTH);
            } else {
                setOpen (x,y,SOUTH);
            }
            if (ifdetectedLeftWall()) {
                 setWall(x,y,EAST);
            } else {
                setOpen (x,y,EAST);
            }
            if (ifdetectedRightWall()) {
                 setWall(x,y,WEST);
            }else {
                setOpen (x,y,WEST);
            } 
            break;
        case WEST:
            if (ifdetectedFrontWall()) {
                 setWall(x,y,WEST);
            } else {
                setOpen (x,y,WEST);
            }
            if (ifdetectedLeftWall()) {
                 setWall(x,y,SOUTH);
            } else {
                setOpen (x,y,SOUTH);
            }
            if (ifdetectedRightWall()) {
                 setWall(x,y,NORTH);
            } else {
                setOpen (x,y,NORTH);
            }
            break;
        case EAST:
            if (ifdetectedFrontWall()) {
                 setWall(x,y,EAST);
            } else {
                setOpen (x,y,EAST);
            }
            if (ifdetectedLeftWall()) {
                 setWall(x,y,NORTH);
            } else {
                setOpen (x,y,NORTH);
            }
            if (ifdetectedRightWall()) {
                 setWall(x,y,SOUTH);
            } else {
                setOpen (x,y,SOUTH);
            }
            break;

    }
   

}
void Maze::moveForward() {


    switch(heading) {
        case NORTH:
            mouseY++;
            break;
        case SOUTH:
            mouseY--;
            break;
        case EAST:
            mouseX++;
            break;
        case WEST:
            mouseX--;
            break;
        case INVALID:
        default:
            break;
    }

}

void Maze::moveBackward() {
    Dir oldHeading = heading;
    heading = opposite(heading);
    moveForward();
    heading = oldHeading;
}

void Maze::move(DriveCommand move) {
    switch(move) {
        case DriveCommand::FORWARD:
            moveForward();
            break;
        case DriveCommand::BACKWARD:
            moveBackward();
            break;
        case DriveCommand::TURNRIGHT:
            turnClockwise();
            break;
        case DriveCommand::TURNLEFT:
            turnCounterClockwise();
            break;
        case DriveCommand::TURN180:
            turnAround();
            break;
        case DriveCommand::NONE:
            // Do nothing, try again
            break;
        default:
            break;
    }
}

MouseMovement Maze::nextMovement() {
    MouseMovement nextMovement;
    
    // char printbuf[128];
    // sprintf(printbuf,"\n\n\nIN MAZE:nextmovement|| wallInFront: %d \n",maze.wallInFront());
    // print((uint8_t*)printbuf);
    discoverWalls();
    // sprintf(printbuf,"IN MAZE:nextmovement after|| heading: %d,wallInFront: %d, wallonleft: %d, wallonRight: %d \n",heading,maze.wallInFront(), maze.wallOnLeft(), maze.wallOnRight());
    // print((uint8_t*)printbuf);
    // printMaze();
    nextMovement = pathFinder->nextMovement(mouseX, mouseY, *this);

    return nextMovement;

}

// void Maze::start() {
//     MouseMovement nextMovement;

//     if(!pathFinder) {
//         return;
//     }

//     while(Finish != (nextMovement = pathFinder->nextMovement(mouseX, mouseY, *this))) {
//         try {
//             switch(nextMovement) {
//                 case MoveForward:
//                     moveForward();
//                     break;
//                 case MoveBackward:
//                     moveBackward();
//                     break;
//                 case TurnClockwise:
//                     turnClockwise();
//                     break;
//                 case TurnCounterClockwise:
//                     turnCounterClockwise();
//                     break;
//                 case TurnAround:
//                     turnAround();
//                     break;
//                 case Wait:
//                     // Do nothing, try again
//                     break;
//                 case Finish:
//                 default:
//                     return;
//             }
//         } catch (std::string str) {
//             std::cerr << str << std::endl;
//         }
//     }
// }

