
#ifndef __DRIVE_H__
#define __DRIVE_H__

#include "IRSensor.hpp"
#include "Encoder.hpp"
#include "PID.hpp"

enum class DriveCommand
{
    FORWARD,
    BACKWARD,
    TURNLEFT,
    TURNRIGHT,
    TURN180,
    NONE
};

extern int pwmL, pwmR;
namespace Command {
	extern DriveCommand cur_command, next_command; 
	extern bool complete; 
	void setNextCommand(); 
};

void goForward(); 
void turnLeft();
void turnRight();
void turn180();
#endif /* __DRIVE_H__ */