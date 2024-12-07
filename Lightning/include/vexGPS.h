#ifndef VEX_GPS_H
#define VEX_GPS_H

#include "pros/gps.hpp"
#include "TaskBase.h"
#include "main.h"
#include "globals.h"
#include "Logger.h"
#include "DiffDrive.h"
#include "MinesMotorGroup.h"

/*put functions and others here
it will be necessary to incorporate functions
to work with driving and the GPS sensor*/

void gpsdrive(double distance, int xdir, int ydir, double speed, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps);
void gpsturn(double tgt, double speed, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps);

#endif

