#include "Drive.hpp"
#include "Utils.hpp"
#include <cmath>

int pwmL, pwmR;

PID motorLPID(20.0,0.35,0.5);
PID motorRPID(20.0,0.35,0.5);
PID encAnglePID(0.02,0.0,0.0);
PID irAnglePID(0.01,0.0,0.0); // 0.002
PID distancePID(0.02,0.0,0.0);

void moveEncoder (float speedTarget, float angleTarget) {
	encAnglePID.setTarget(angleTarget);
	float encAnglePIDResult = encAnglePID.update(EncAngle);
	float speedW = abs(encAnglePIDResult) < V_MAX ? encAnglePIDResult : sgn(encAnglePIDResult) * V_MAX;

	motorLPID.setTarget(speedTarget - speedW);
	motorRPID.setTarget(speedTarget + speedW);

}

void moveIR(float speedTarget, float irError) {

	irAnglePID.setTarget(0.0);
	float irPIDResult = irAnglePID.update(irError);
	float speedW = abs(irPIDResult) < V_MAX ? irPIDResult : sgn(irPIDResult) * V_MAX;

	motorLPID.setTarget(speedTarget - speedW);
	motorRPID.setTarget(speedTarget + speedW);
}


void move(float speedTarget, float angleTarget) {

	float irError = 0.0;

	// if too close to right wall 
	if ((IRTopRight.value() - WALL_R > 20) && (angleTarget == 0)) {
		irError = WALL_R - IRTopRight.value(); 
		moveIR(speedTarget, irError);
	} else if ((IRTopLeft.value() - WALL_L > 20) && (angleTarget == 0)) {
		irError = IRTopLeft.value() - WALL_L;
		moveIR(speedTarget, irError);
	} else {
		// no correction use encoder 
		moveEncoder(speedTarget, angleTarget);
	}

	// char printbuf[64];
	// sprintf(printbuf,"Err: %d\r\n", (int)irError);
	// print((uint8_t*)printbuf);
	pwmL = motorLPID.update(diffL);
	pwmR = motorRPID.update(diffR);

	if (speedTarget == 0.0 && abs(encAnglePID.getError()) < 45) {
		Command::complete = true;
	}
}


void goForward(int cellCount) {
	if (ifdetectedFrontWall()) {
		adjustFront();
		return;
	}
	else{
		distancePID.setTarget(cellCount * CELL);
		float temp = distancePID.update(EncAvg);
		float speed = temp > 15? 15 : temp;
		move(speed,0.0);
	}
	if(abs(distancePID.getError()) < 100)
	{
		Command::complete = true;
	}

}
void turnLeft(){
	move(0,5000);
	if (EncAngle >= 5000) Command::complete = true;
}

void turnRight(){
	move(0,-5000);
	if (EncAngle <= -5000) Command::complete = true;
}
void turn180(){
	move(0,10500);
	if (EncAngle >= 10500) Command::complete = true;
}

void adjustFront() {
	if(IRLeft.value() > WALL_F){
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		moveEncoder(-5.0, 0.0);
		pwmL = motorLPID.update(diffL);
		pwmR = motorRPID.update(diffR);
	} else {
		Command::complete = true;
	}
	return;
}

namespace Command {
	etl::queue<DriveCommand, 255, etl::memory_model::MEMORY_MODEL_SMALL> Q;
	bool complete = true; 
};

void Command::setNextCommand() {

	if (!ifdetectedFrontWall()) {
		Q.push(DriveCommand::FORWARD);
	} else if (!ifdetectedLeftWall()) {
		Q.push(DriveCommand::TURNLEFT);
		Q.push(DriveCommand::FORWARD);
	} else if (!ifdetectedRightWall()) {
		Q.push(DriveCommand::TURNRIGHT);
		Q.push(DriveCommand::FORWARD);
	} else {
		Q.push(DriveCommand::TURN180);
		Q.push(DriveCommand::FORWARD);
	}
}