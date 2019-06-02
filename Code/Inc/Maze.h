#ifndef Maze_h
#define Maze_h


#include "BitVector256.h"
#include "MazeDefinitions.h"
#include "Dir.h"
#include "PathFinder.h"
#include "Drive.hpp"
#include "Floodfill.hpp"


class Maze {
public:
    BitVector256 wallNS;
    BitVector256 wallEW;
    Dir heading;
    PathFinder *pathFinder;
    unsigned mouseX;
    unsigned mouseY;


    bool isOpen(unsigned x, unsigned y, Dir d) const;
    void setOpen(unsigned x, unsigned y, Dir d);
    void setWall(unsigned x, unsigned y, Dir d);

    void moveForward();
    void moveBackward();
    void discoverWalls();
    void move(DriveCommand move);
    void printMaze();
    inline void turnClockwise() {
        heading = clockwise(heading);
    }

    inline void turnCounterClockwise() {
        heading = counterClockwise(heading);
    }

    inline void turnAround() {
        heading = opposite(heading);
    }

// public:
    Maze(PathFinder *pathFinder);

    inline bool wallInFront() const {
        return !isOpen(mouseX, mouseY, heading);
    }

    inline bool wallOnLeft() const {
        return !isOpen(mouseX, mouseY, counterClockwise(heading));
    }

    inline bool wallOnRight() const {
        return !isOpen(mouseX, mouseY, clockwise(heading));
    }

    MouseMovement nextMovement();
    /**
     * Start running the mouse through the maze.
     * Terminates when the PathFinder's nextMovement method returns MouseMovement::Finish.
     */
    void start();

    /**
     * This function draws the maze using ASCII characters.
     *
     * Queries the underlying PathFinder for additional maze info
     * and incorporates it in the maze rendering.
     * @param infoLen: specifies the max characters of info to be drawn. If no info is supplied, blank spaces will be inserted.
     * @return string of rendered maze
     */
};
extern Maze maze;
#endif
