#ifndef BOTFUNCTIONS_H
#define BOTFUNCTIONS_H

#include "globals.h"
#include "MinesMotorGroup.h"
#include "pros/motors.h"

void driveLoop(Mines::MinesMotorGroup leftMotorGroup, Mines::MinesMotorGroup rightMotorGroup, double leftVelocity, double rightVelocity);
void catLaunch(Mines::MinesMotorGroup cataMotors, double velocity);
void catPrime(Mines::MinesMotorGroup cataMotors, pros::ADIDigitalIn limitSwitch, double velocity);
//Color getColor(pros::c::optical_rgb_s_t color);
//void swapRollerColor(Color color, double voltage);

#endif