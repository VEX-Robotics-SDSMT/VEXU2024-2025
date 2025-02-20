#ifndef _GLOBALS_H
#define _GLOBALS_H

#include "api.h"
#include "MinesMotorGroup.h"
#include "pros/adi.hpp"

#define INERTIAL_SENSOR 17
#define TRACKING_TOP 3
#define TRACKING_BOT 4 

#define L_FRONT 12
#define L_MID 13
#define L_REAR 3
#define R_FRONT 20
#define R_MID 10
#define R_REAR 18
#define U_INTAKE 1
#define L_INTAKE 19

#define ARM 15
#define LIFT 2
#define MOGO 1

extern pros::Controller MasterController;

extern pros::Imu intertialSensor;
extern pros::ADIEncoder tracking;

extern pros::Motor leftFront;
extern pros::Motor leftMid;
extern pros::Motor leftRear;
extern pros::Motor rightFront;
extern pros::Motor rightMid;
extern pros::Motor rightRear;
extern pros::Motor intake;
extern pros::Motor convR;
extern pros::Motor convL;
extern pros::Motor arm;

extern std::vector<pros::Motor> leftDriveVector;
extern std::vector<pros::Motor> rightDriveVector;
extern Mines::MinesMotorGroup leftDriveMotors;
extern Mines::MinesMotorGroup rightDriveMotors;
extern Mines::MinesMotorGroup intakeMotors;

extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut lift;


extern pros::Motor string;

extern double axisPercentBlue;
extern double axisPercentGreen;
extern double axisPercentRed;
extern int blueGearbox;
extern int greenGearbox;
extern int redGearbox;

extern bool skills;
extern bool red;

extern uint8_t RED_GOAL_SIG_ID;
extern uint8_t BLUE_GOAL_SIG_ID;

#define buttonUp pros::E_CONTROLLER_DIGITAL_UP
#define buttonDown pros::E_CONTROLLER_DIGITAL_DOWN
#define buttonLeft pros::E_CONTROLLER_DIGITAL_LEFT
#define buttonRight pros::E_CONTROLLER_DIGITAL_RIGHT
#define buttonX pros::E_CONTROLLER_DIGITAL_X
#define buttonY pros::E_CONTROLLER_DIGITAL_Y
#define buttonA pros::E_CONTROLLER_DIGITAL_A
#define buttonB pros::E_CONTROLLER_DIGITAL_B
#define buttonL1 pros::E_CONTROLLER_DIGITAL_L1
#define buttonL2 pros::E_CONTROLLER_DIGITAL_L2
#define buttonR1 pros::E_CONTROLLER_DIGITAL_R1
#define buttonR2 pros::E_CONTROLLER_DIGITAL_R2
#define axisLeftY pros::E_CONTROLLER_ANALOG_LEFT_Y
#define axisLeftX pros::E_CONTROLLER_ANALOG_LEFT_X
#define axisRightY pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define axisRightX pros::E_CONTROLLER_ANALOG_RIGHT_X

#endif