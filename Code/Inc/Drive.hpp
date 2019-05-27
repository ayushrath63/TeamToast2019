
#ifndef __DRIVE_H__
#define __DRIVE_H__

#include "main.h"
#include "IRSensor.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include <etl/queue.h>

enum class DriveCommand : uint8_t
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
	extern etl::queue<DriveCommand, 255, etl::memory_model::MEMORY_MODEL_SMALL> Q;
	extern bool complete; 
	void setNextCommand(); 
};

void goForward(); 
void turnLeft();
void turnRight();
void turn180();
#endif /* __DRIVE_H__ */