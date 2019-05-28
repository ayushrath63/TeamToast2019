#include "Drive.hpp"
#include <cmath>
// Drive::Drive(IMU* imu, Motor* motorL, Motor* motorR, PID* motorLPID, PID* motorLPID){
// :m_imu(imu), m_motorL(motorL), m_motorR(motorR), m_motorLPID (motorLPID), m_motorRPID (motorRPID)
// 	m_imuSum = 0 
// }
// void turnRight() {
// 	m_imuSum = 0 
// 	if (m_imuSum >= -3000 ) {
//       motorR->setSpeed(rSpeed);
//       motorL->setSpeed(lSpeed);
//     }
//     else {
//       motorR->setSpeed(0);
//       motorL->setSpeed(0);
//     }
// }
// void turnLeft() {
// 	m_imuSum = 0 
// 	if (m_imuSum <= 3000 ) {
//       motorR->setSpeed(rSpeed);
//       motorL->setSpeed(lSpeed);
//     }
//     else {
//       motorR->setSpeed(0);
//       motorL->setSpeed(0);
//     }
    
// }
int pwmL, pwmR;
const int CELL = 4400; 

PID motorLPID(20.0,0.35,0.5);
PID motorRPID(20.0,0.35,0.5);
PID encAnglePID(0.02,0.0,0.0);
PID distancePID(0.1,0.0,0.0);

void move(float speedTarget, float angleTarget) {
	encAnglePID.setTarget(angleTarget);
	float encAnglePIDResult = encAnglePID.update(EncAngle);
	float speedW = abs(encAnglePIDResult) < 15 ? encAnglePIDResult : encAnglePIDResult / abs(encAnglePIDResult) *15;
	motorLPID.setTarget(speedTarget - speedW);
	motorRPID.setTarget(speedTarget + speedW);
	pwmL = motorLPID.update(diffL);
	pwmR = motorRPID.update(diffR);

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
	if (EncAvg >= CELL) Command::complete = true;
	
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
	DriveCommand cur_command = DriveCommand::FORWARD, next_command = DriveCommand::NONE; 
	bool complete = false; 
};
void Command::setNextCommand() {
	 char gzbuf[128];
    sprintf(gzbuf,"SEND NEXT COMMAND");
    print((uint8_t*)gzbuf);
    int cmd = 0; 
    int num = rand();
    if (!ifdetectedLeftWall() &&!ifdetectedRightWall() && !ifdetectedFrontWall()) {
    	cmd = num%3;
    } else if (!ifdetectedLeftWall() &&!ifdetectedRightWall() ) {
    	cmd = num%2 +1;
    } else if (!ifdetectedLeftWall() &&!ifdetectedFrontWall() ) {
    	cmd = num%2;
    } else if (!ifdetectedRightWall() &&!ifdetectedFrontWall()) {
    	cmd = num%2;
    	cmd = cmd *2;
    } else if (!ifdetectedRightWall()) {
    	cmd = 2;
    } else if (!ifdetectedLeftWall()) {
    	cmd = 1;
    } else if (!ifdetectedFrontWall()) {
    	cmd = 0;
    } else {
    	cmd = 3;
    }
    switch(cmd) {
    	case 0: // forward
	    	cur_command = DriveCommand::FORWARD;
			next_command = DriveCommand::NONE;
			break;
		case 1: // Left
			cur_command = DriveCommand::TURNLEFT; 
			next_command = DriveCommand::FORWARD;
			break;
		case 2: 
			cur_command =  DriveCommand::TURNRIGHT; 
			next_command = DriveCommand::FORWARD;
			break;
		default:
			cur_command = DriveCommand::TURN180;
			next_command = DriveCommand::FORWARD;
			break;

    }

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
	

}