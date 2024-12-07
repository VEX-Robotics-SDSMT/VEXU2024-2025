#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::Vision vision(VISION_SENSOR);
pros::ADIEncoder driveEncoder(ENCODER_TOP, ENCODER_BOTTOM, true);
pros::GPS gps(VEX_GPS);

//RED Motors E_MOTOR_GEARSET_36
//GREEN Motors E_MOTOR_GEARSET_18
//BLUE Motors E_MOTOR_GEARSET_06

pros::Motor leftFront(L_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftMid(L_MID, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftRear(L_REAR, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFront(R_FRONT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightMid(R_MID, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightRear(R_REAR, pros::E_MOTOR_GEARSET_06, false);
pros::Motor intake(INTAKE, pros::E_MOTOR_GEARSET_18, true);
pros::Motor convL(COVEL, pros::E_MOTOR_GEARSET_06, true);
pros::Motor convR(COVER, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lift_r(LIFTR, pros::E_MOTOR_GEARSET_36, true);
pros::Motor lift_l(LIFTL, pros::E_MOTOR_GEARSET_36, false);

std::vector<pros::Motor> leftDriveVector = {leftFront, leftMid, leftRear};
std::vector<pros::Motor> rightDriveVector = {rightFront, rightMid, rightRear};
std::vector<pros::Motor> conveyorVector = {convL, convR};
std::vector<pros::Motor> liftVector = {lift_r, lift_l};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);
Mines::MinesMotorGroup conveyorMotors(conveyorVector);
Mines::MinesMotorGroup liftMotors(liftVector);

pros::ADIDigitalOut mogo(MOGO);

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