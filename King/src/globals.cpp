#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::ADIEncoder driveEncoder(ENCODER_TOP_L, ENCODER_BOTTOM_L, true);
pros::ADIDigitalIn limitSwitch(LIMIT_SWITCH);

//pros::GPS gps(VEX_GPS);

//RED Motors E_MOTOR_GEARSET_36
//GREEN Motors E_MOTOR_GEARSET_18
//BLUE Motors E_MOTOR_GEARSET_06

pros::Motor leftFront(L_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftMid(L_MID, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftMid2(L_MID2, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftRear(L_REAR, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFront(R_FRONT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightMid(R_MID, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightMid2(R_MID2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightRear(R_REAR, pros::E_MOTOR_GEARSET_06, false);
pros::Motor intake(INTAKE, pros::E_MOTOR_GEARSET_18, false);
pros::Motor convL(CONV_L, pros::E_MOTOR_GEARSET_06, true);
pros::Motor convR(CONV_R, pros::E_MOTOR_GEARSET_06, false);
pros::Motor arm(ARM, pros::E_MOTOR_GEARSET_36, true);

std::vector<pros::Motor> leftDriveVector = {leftFront, leftMid, leftMid2, leftRear};
std::vector<pros::Motor> rightDriveVector = {rightFront, rightMid, rightMid2, rightRear};
std::vector<pros::Motor> conveyorVector = {convL, convR};
std::vector<pros::Motor> intakeVector = {intake, convL, convR};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);
Mines::MinesMotorGroup conveyorMotors(conveyorVector);
Mines::MinesMotorGroup intakeMotors(intakeVector); //combines conveyor and intake
pros::ADIDigitalOut mogo(MOGO);
pros::ADIDigitalOut wing(WING);

double axisPercentBlue = 600.0 / 127;
double axisPercentGreen = 200.0 / 127;
double axisPercentRed = 100.0 / 127;
int blueGearbox = 600;
int greenGearbox = 200;
int redGearbox = 100;


uint8_t RED_GOAL_SIG_ID = 1;
uint8_t BLUE_GOAL_SIG_ID = 2;

int requiredColorLoops = 3;
const double ROLLER_TIMEOUT = 3000;

bool skills = true;
bool red_team = true;