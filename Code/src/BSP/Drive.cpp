#include "Drive.hpp"
#include "Utils.hpp"
#include <cmath>

int pwmL, pwmR;
const int CELL = 4450;

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
	distancePID.setTarget(cellCount * CELL);
	float temp = distancePID.update(EncAvg);
	float speed = temp > 15? 15 : temp;
	move(speed,0.0);

	if(abs(distancePID.getError()) < 100)
	{
		Command::complete = true;
	}

}
void turnLeft(){
	move(0,5250);
	if (EncAngle >= 5250) Command::complete = true;
}
void turnRight(){
	move(0,-5250);
	if (EncAngle <= -5250) Command::complete = true;
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

	 // char gzbuf[128];
  //   sprintf(gzbuf,"SEND NEXT COMMAND");
  //   print((uint8_t*)gzbuf);
  //   int cmd = 0; 
  //   int num = rand();
  //   if (!ifdetectedLeftWall() &&!ifdetectedRightWall() && !ifdetectedFrontWall()) {
  //   	cmd = num%3;
  //   } else if (!ifdetectedLeftWall() &&!ifdetectedRightWall() ) {
  //   	cmd = num%2 +1;
  //   } else if (!ifdetectedLeftWall() &&!ifdetectedFrontWall() ) {
  //   	cmd = num%2;
  //   } else if (!ifdetectedRightWall() &&!ifdetectedFrontWall()) {
  //   	cmd = num%2;
  //   	cmd = cmd *2;
  //   } else if (!ifdetectedRightWall()) {
  //   	cmd = 2;
  //   } else if (!ifdetectedLeftWall()) {
  //   	cmd = 1;
  //   } else if (!ifdetectedFrontWall()) {
  //   	cmd = 0;
  //   } else {
  //   	cmd = 3;
  //   }
  //   switch(cmd) {
  //   	case 0: // forward
	 //    	cur_command = DriveCommand::FORWARD;
		// 	next_command = DriveCommand::NONE;
		// 	break;
		// case 1: // Left
		// 	cur_command = DriveCommand::TURNLEFT; 
		// 	next_command = DriveCommand::FORWARD;
		// 	break;
		// case 2: 
		// 	cur_command =  DriveCommand::TURNRIGHT; 
		// 	next_command = DriveCommand::FORWARD;
		// 	break;
		// default:
		// 	cur_command = DriveCommand::TURN180;
		// 	next_command = DriveCommand::FORWARD;
		// 	break;

  //   }

	// if (!ifdetectedFrontWall() && (rand()%3)) {
	// 	cur_command = DriveCommand::FORWARD;
	// 	next_command = DriveCommand::NONE;
	// } else if (!ifdetectedLeftWall()&& (rand()%2)) {
	// 	cur_command = DriveCommand::TURNLEFT; 
	// 	next_command = DriveCommand::FORWARD;
	// } else if (!ifdetectedRightWall()&& (rand()%2)) {
	// 	cur_command =  DriveCommand::TURNRIGHT; 
	// 	next_command = DriveCommand::FORWARD;
	// } else {
	// 	cur_command = DriveCommand::TURN180;
	// 	next_command = DriveCommand::FORWARD;
	// }

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