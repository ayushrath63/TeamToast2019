// #include "Drive.hpp"

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