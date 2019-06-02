
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

constexpr float V_MAX = 10.0; // tick/ms
constexpr int CELL = 4550;

extern int pwmL, pwmR;
extern PID motorLPID, motorRPID, encAnglePID, irAnglePID, distancePID;

namespace Command {
	extern etl::queue<DriveCommand, 255, etl::memory_model::MEMORY_MODEL_SMALL> Q;
	extern bool complete; 
	DriveCommand setNextCommand(); 
};

void goForward(int cellCount = 1); 
void turnLeft();
void turnRight();
void turn180();
void adjustFront();
#endif /* __DRIVE_H__ */