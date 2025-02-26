#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::ADIEncoder tracking(TRACKING_TOP, TRACKING_BOT, false);

pros::Motor leftFront(L_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftMid(L_MID, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftRear(L_REAR, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFront(R_FRONT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightMid(R_MID, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightRear(R_REAR, pros::E_MOTOR_GEARSET_06, false);
pros::Motor arm(ARM, pros::E_MOTOR_GEARSET_36, true);
pros::Motor intakeU(U_INTAKE, pros::E_MOTOR_GEARSET_36, false);
pros::Motor intakeL(L_INTAKE, pros::E_MOTOR_GEARSET_36, true);

std::vector<pros::Motor> leftDriveVector = {leftFront, leftMid, leftRear};
std::vector<pros::Motor> rightDriveVector = {rightFront, rightMid, rightRear};
std::vector<pros::Motor> intakeVector = {intakeU, intakeL};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);
Mines::MinesMotorGroup intakeMotors(intakeVector);

pros::ADIDigitalOut mogo(MOGO);
pros::ADIDigitalOut lift(LIFT);


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

bool skills = false;
bool red = false;