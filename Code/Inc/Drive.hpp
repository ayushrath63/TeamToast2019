
#ifndef __DRIVE_H__
#define __DRIVE_H__



enum class DriveCommand
{
    FORWARD,
    BACKWARD,
    TURNLEFT,
    TURNRIGHT,
    TURN180,
    NONE
};


namespace Command {
	extern DriveCommand cur_command, next_command; 
	extern bool complete; 
	void setNextCommand(); 
};

void goForward(); 
#endif /* __DRIVE_H__ */