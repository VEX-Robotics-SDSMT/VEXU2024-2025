#ifndef _GLOBALS_H
#define _GLOBALS_H

#include "api.h"
#include "MinesMotorGroup.h"
#include "pros/adi.hpp"

#define INERTIAL_SENSOR 2
#define VISION_SENSOR 4
#define VEX_GPS 21
#define ENCODER_TOP_R 1
#define ENCODER_BOTTOM_R 2
#define ENCODER_TOP_L 3
#define ENCODER_BOTTOM_L 4

#define L_FRONT 13
#define L_MID 14
#define L_REAR 11
#define R_FRONT 20
#define R_MID 12
#define R_REAR 15
#define CONV_R 9
#define CONV_L 8
#define INTAKE 19
#define ARM 17

#define MOGO 8
#define WING 7


#define INTAKE_MOTOR_GEARSET redGearbox
#define FLYWHEELS_MOTOR_GEARSET blueGearbox
#define ROLLER_MOTOR_GEARSET greenGearbox

extern pros::Controller MasterController;

extern pros::Imu intertialSensor;
extern pros::Vision vision;
extern pros::ADIEncoder driveEncoderL;
extern pros::ADIEncoder driveEncoderR;
//extern pros::GPS gps;

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