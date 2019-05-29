#include "Drive.hpp"
#include "Utils.hpp"
#include <cmath>

int pwmL, pwmR;
const int CELL = 4400;

PID motorLPID(20.0,0.35,0.5);
PID motorRPID(20.0,0.35,0.5);
PID encAnglePID(0.02,0.0,0.0);
PID irAnglePID(0.00,0.0,0.0); // 0.002
PID distancePID(0.02,0.0,0.0);

void moveEncoder (float speedTarget, float angleTarget) {
	encAnglePID.setTarget(angleTarget);
	float encAnglePIDResult = encAnglePID.update(EncAngle);
	float speedW = abs(encAnglePIDResult) < 10 ? encAnglePIDResult : sgn(encAnglePIDResult) * 10;

	motorLPID.setTarget(speedTarget - speedW);
	motorRPID.setTarget(speedTarget + speedW);

}
void moveIR(float speedTarget) {
	float irError = 0.0, irPIDResult = 0.0;
	if(ifdetectedRightWall()) {
		irError = -1*(IRTopRight.value()-WALL_R);
	} else if(ifdetectedLeftWall()) {
		irError = (WALL_L - IRTopLeft.value());
	} 



	irAnglePID.setTarget(0.0);
	irPIDResult = irAnglePID.update(irError);
	float speedW = abs(irPIDResult) < 10 ? irPIDResult : sgn(irPIDResult) * 10;
	

	char printbuf[128];
	sprintf(printbuf,"irError %d, irPIDResult:%d",(int)(irError), (int)(irPIDResult));
    print((uint8_t*)printbuf);

	motorLPID.setTarget(speedTarget - speedW);
	motorRPID.setTarget(speedTarget + speedW);
}


void move(float speedTarget, float angleTarget) {

	if( (ifdetectedRightWall() || ifdetectedLeftWall()) && (angleTarget == 0) ) {
		moveIR(speedTarget);
	} else {
		moveEncoder(speedTarget, angleTarget);
	}

	pwmL = motorLPID.update(diffL);
	pwmR = motorRPID.update(diffR);

	if (speedTarget == 0.0 && abs(encAnglePID.getError()) < 45) {
		Command::complete = true;
	}
}


void goForward() {
	if (ifdetectedFrontWall()) {
		Command::complete = true;
		return;
	}
	distancePID.setTarget(CELL);
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
	char printbuf[50];
	sprintf(printbuf,"if front wall: %d", ifdetectedFrontWall());
    print((uint8_t*)printbuf);

	if (!ifdetectedFrontWall()) {
		sprintf(printbuf,"NO WALL");
    	print((uint8_t*)printbuf);
		Q.push(DriveCommand::FORWARD);
	} else if (!ifdetectedLeftWall()) {
		sprintf(printbuf,"L\r\n");
		print((uint8_t*)printbuf);
		Q.push(DriveCommand::TURNLEFT);
		Q.push(DriveCommand::FORWARD);
	} else if (!ifdetectedRightWall()) {
		sprintf(printbuf,"R\r\n");
		print((uint8_t*)printbuf);
		Q.push(DriveCommand::TURNRIGHT);
		Q.push(DriveCommand::FORWARD);
	} else {
		sprintf(printbuf,"180\r\n");
		print((uint8_t*)printbuf);
		Q.push(DriveCommand::TURN180);
		Q.push(DriveCommand::FORWARD);
	}
}