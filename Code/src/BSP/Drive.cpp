#include "Drive.hpp"
#include "Utils.hpp"
#include <cmath>

int pwmL, pwmR;
const int CELL = 4400;

PID motorLPID(20.0,0.35,0.5);
PID motorRPID(20.0,0.35,0.5);
PID encAnglePID(0.02,0.0,0.0);
PID irAnglePID(0.02,0.0,0.0);
PID distancePID(0.02,0.0,0.0);

void move(float speedTarget, float angleTarget) {
	// if(ifdetectedLeftWall() && ifdetectedRightWall()) {
	// 	irError = IRTopRight.value() - IRTopLeft.value() + 0; //change 0 to offset
	// } else 
	float irError = 0.0, irPIDResult = 0.0;
	if(angleTarget == 0.0)
	{
		if(ifdetectedRightWall()) {
			irError = (IRTopRight.value()-WALL_R);
		} else if(ifdetectedLeftWall()) {
			irError = (WALL_L - IRTopLeft.value());
		} 
		irAnglePID.setTarget(0.0);
		irPIDResult = irAnglePID.update(irError);
	}
	
	encAnglePID.setTarget(angleTarget);
	float encAnglePIDResult = encAnglePID.update(EncAngle);
	float anglePIDResultSum = irPIDResult + encAnglePIDResult;
	float speedW = abs(anglePIDResultSum) < 10 ? anglePIDResultSum : sgn(anglePIDResultSum) * 10;
	motorLPID.setTarget(speedTarget - speedW);
	motorRPID.setTarget(speedTarget + speedW);
	pwmL = motorLPID.update(diffL);
	pwmR = motorRPID.update(diffR);
	if (speedTarget == 0.0 && abs(encAnglePID.getError()) < 45) {
		Command::complete = true;
	}
}

void goForward() {

	distancePID.setTarget(CELL);
	float temp = distancePID.update(EncAvg);
	float speed = temp > 10? 10 : temp;
	move(speed,0.0);
	if(abs(distancePID.getError()) < 100)
	{
		Command::complete = true;
	}
	
}
void turnLeft(){
	move(0,5250);
}
void turnRight(){
	move(0,-5250);
}
void turn180(){
	move(0,10500);
}

namespace Command {
	etl::queue<DriveCommand, 255, etl::memory_model::MEMORY_MODEL_SMALL> Q;
	bool complete = false; 
};

void Command::setNextCommand() {
	char printbuf[128];
	sprintf(printbuf,"SENDING ");
	print((uint8_t*)printbuf);
	if (!ifdetectedFrontWall()) {
		sprintf(printbuf,"F\r\n");
		print((uint8_t*)printbuf);
		Q.push(DriveCommand::FORWARD);
	} else if (!ifdetectedLeftWall()) {
				sprintf(printbuf,"L\r\n");
		print((uint8_t*)printbuf);
		Q.push(DriveCommand::TURNLEFT);
//		Q.push(DriveCommand::FORWARD);
	} else if (!ifdetectedRightWall()) {
				sprintf(printbuf,"R\r\n");
		print((uint8_t*)printbuf);
		Q.push(DriveCommand::TURNRIGHT);
//		Q.push(DriveCommand::FORWARD);
	} else {
		Q.push(DriveCommand::TURN180);
//		Q.push(DriveCommand::FORWARD);
	}
}