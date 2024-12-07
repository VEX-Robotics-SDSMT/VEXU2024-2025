#include "../include/botFunctions.h"

bool intakeToggle = 0;
bool flywheelToggle = 0;
bool mogoToggle = 0;

void toggleIntake()
{
    if(intakeToggle == 0)
        intakeToggle = 1;
    else
        intakeToggle = 0;
}

void toggleFlywheels()
{
    if(flywheelToggle == 0)
        flywheelToggle = 1;
    else
        flywheelToggle = 0;
}

void toggleMOGO()
{
    if(mogoToggle == 0)
        mogoToggle = 1;
    else
        mogoToggle = 0;
}

void driveLoop(Mines::MinesMotorGroup leftMotorGroup, Mines::MinesMotorGroup rightMotorGroup, double leftVelocity, double rightVelocity)
{
    leftMotorGroup.move(leftVelocity);
    rightMotorGroup.move(rightVelocity);
}

