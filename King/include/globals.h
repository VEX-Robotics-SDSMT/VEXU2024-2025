#ifndef _GLOBALS_H
#define _GLOBALS_H

#include "api.h"
#include "MinesMotorGroup.h"
#include "pros/adi.hpp"

#define INERTIAL_SENSOR 18
#define ENCODER_TOP_L 6
#define ENCODER_BOTTOM_L 7

#define L_FRONT 19
#define L_MID 16
#define L_MID2 17
#define L_REAR 15
#define R_FRONT 4
#define R_MID 1
#define R_MID2 7
#define R_REAR 5

#define CONV_R 3
#define CONV_L 13
#define INTAKE 20
#define ARM 10

#define MOGO 1
#define WING 2


#define INTAKE_MOTOR_GEARSET redGearbox
#define FLYWHEELS_MOTOR_GEARSET blueGearbox
#define ROLLER_MOTOR_GEARSET greenGearbox

extern pros::Controller MasterController;

extern pros::Imu intertialSensor;
extern pros::ADIEncoder driveEncoder;
extern pros::GPS gps;

extern pros::Motor leftFront;
extern pros::Motor leftMid;
extern pros::Motor leftMid2;
extern pros::Motor leftRear;
extern pros::Motor rightFront;
extern pros::Motor rightMid;
extern pros::Motor rightMid2;
extern pros::Motor rightRear;
extern pros::Motor intake;
extern pros::Motor convR;
extern pros::Motor convL;
extern pros::Motor arm;

extern std::vector<pros::Motor> leftDriveVector;
extern std::vector<pros::Motor> rightDriveVector;
extern std::vector<pros::Motor> conveyorVector;
extern std::vector<pros::Motor> intakeVector;
extern Mines::MinesMotorGroup leftDriveMotors;
extern Mines::MinesMotorGroup rightDriveMotors;
extern Mines::MinesMotorGroup conveyorMotors;
extern Mines::MinesMotorGroup intakeMotors;

extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut wing;

enum Color { red, blue, purple };
extern pros::Motor string;

extern double axisPercentBlue;
extern double axisPercentGreen;
extern double axisPercentRed;
extern int blueGearbox;
extern int greenGearbox;
extern int redGearbox;

extern bool skills;
extern bool red_team;

extern uint8_t RED_GOAL_SIG_ID;
extern uint8_t BLUE_GOAL_SIG_ID;

extern int requiredColorLoops;
extern const double ROLLER_TIMEOUT;

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