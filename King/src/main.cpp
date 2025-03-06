#include "../include/main.h"
#include "DiffDrive.h"
#include "botFunctions.h"
#include "globals.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

//globals

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

using namespace Mines;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	//intertialSensor.reset();
	//pros::vision_signature_s_t RED_GOAL_SIG = vision.signature_from_utility(1, 4391, 7505, 5948, -1303, -147, -725, 1.6, 0);
	//vision.set_signature(RED_GOAL_SIG_ID, &RED_GOAL_SIG);
	//pros::vision_signature_s_t BLUE_GOAL_SIG = vision.signature_from_utility(2, -3073, -1323, -2198, 4405, 9923, 7164, 1.5, 0);
	//vision.set_signature(BLUE_GOAL_SIG_ID, &BLUE_GOAL_SIG);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	//Skills?
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{
	//PID Setup
	EncoderWheelSensorInterface encoderInterface(driveEncoder);
	//EncoderWheelSensorInterface encoderInterface(driveEncoderR);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	drive.setDrivePIDVals(0.85, 0, 1); //0.75 Tuned 2/1/2024
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(4.4, 0, 0); //4.25 Tuned 2/1/2024
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.75); 
	drive.setMaxTurnSpeed(0.8);
	drive.setMaxDriveAccel(0.12);

	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// drive.turnDegreesAbsolute(180); // PID TUNING 
	// drive.turnDegreesAbsolute(0);
	// drive.driveTiles(1000);
	// drive.driveTiles(-1000);
	// drive.turnDegreesAbsolute(180);
	// drive.turnDegreesAbsolute(0);
	// drive.killPIDs();	
	
	if (skills)  // Skills
	{
		drive.setMaxDriveSpeed(0.6); 
		intakeMotors.move(600);
		drive.driveTiles(250);
		drive.driveTiles(-225);
		pros::delay(1000);
		drive.driveTiles(225);
		drive.turnDegreesAbsolute(-90);
		drive.driveTiles(1200);
		intakeMotors.brake();
		intake.move(127);
		conveyorMotors.move(50);

		drive.turnDegreesAbsolute(0);
		drive.driveTiles(1525);
		drive.turnDegreesAbsolute(90);
		intakeMotors.brake();
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-950);
		mogo.set_value(1);
		intakeMotors.move(127);

		drive.setMaxDriveSpeed(0.6);
		drive.turnDegreesAbsolute(0);
		drive.driveTiles(950);
		drive.turnDegreesAbsolute(-90);
		drive.driveTiles(350);
		drive.driveTiles(-350);
		drive.turnDegreesAbsolute(180);
		drive.driveTiles(2350);
		drive.driveTiles(-250);

		drive.turnDegreesAbsolute(-135); // turn towards corner
		drive.driveTiles(850, 2000);
		drive.driveTiles(-550);
		drive.turnDegreesAbsolute(45);
		mogo.set_value(0);
		drive.setMaxDriveSpeed(.3);
		drive.driveTiles(-300, 1000);
		drive.setMaxDriveSpeed(.6);
		drive.driveTiles(675);

		drive.turnDegreesAbsolute(0);
		drive.driveTiles(3250);
		intakeMotors.brake();
		intake.move(127);
		conveyorMotors.move(45);

		drive.turnDegreesAbsolute(-90);
		conveyorMotors.brake();
		drive.setMaxDriveSpeed(.3);
		drive.driveTiles(-1200);
		mogo.set_value(1);
		drive.setMaxDriveSpeed(.6);
		drive.turnDegreesAbsolute(-45);
		drive.driveTiles(400);
		intakeMotors.move(127);
		drive.driveTiles(1300);
		intakeMotors.brake();
		conveyorMotors.move(127);
		intake.move(-600);
		wing.set_value(1);
		drive.turnDegreesAbsolute(-100);
		wing.set_value(0);
		drive.turnDegreesAbsolute(120);
		drive.turnDegreesAbsolute(155);
		intakeMotors.brake();
		mogo.set_value(0);
		drive.setMaxDriveSpeed(.3);
		drive.driveTiles(-400, 1500);
		drive.setMaxDriveSpeed(.6);
		drive.driveTiles(675);
		drive.turnDegreesAbsolute(90);
		while(limitSwitch.get_value())
			arm.move(-127);
		pros::delay(5);
		arm.brake();
		intakeMotors.move(90);
		drive.driveTiles(1000);
		drive.turnDegreesAbsolute(52);
		pros::delay(2000);
		intakeMotors.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		intakeMotors.brake();
		drive.driveTiles(925);
		arm.move(-127);
		pros::delay(750);
		arm.move(127);
		pros::delay(200);
		arm.brake();
		drive.driveTiles(-500);				
	}
	else // Red auto
	{
		drive.driveTiles(-2530);
		mogo.set_value(1);
		drive.driveTiles(1580);
		mogo.set_value(0);
		drive.driveTiles(200);
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-250);
		mogo.set_value(1);
		drive.driveTiles(50);
		drive.setMaxDriveSpeed(0.75);
		drive.turnDegreesAbsolute(95);
		drive.setMaxDriveSpeed(.5);
		intakeMotors.move(600);
		drive.driveTiles(850);
		drive.driveTiles(-850);
		drive.turnDegreesAbsolute(155);
		while(limitSwitch.get_value())
			arm.move(-127);
		arm.move(127);
		pros::delay(85);
		arm.brake();
		drive.driveTiles(1200);
		pros::delay(2000);
		arm.move(127);
		pros::delay(100);
		arm.brake();
		drive.driveTiles(500);
		drive.driveTiles(-1700);
		drive.turnDegreesAbsolute(88);
		wing.set_value(1);
		intakeMotors.brake();
		drive.driveTiles(1500, 1500);
		drive.turnDegreesAbsolute(85);
		wing.set_value(0);
		drive.turnDegreesAbsolute(262);
		if(!red_team)
		{
			drive.setMaxDriveSpeed(0.3);
			mogo.set_value(0);
			drive.driveTiles(-500, 1000);
			drive.driveTiles(300);
			drive.setMaxDriveSpeed(0.75);
		}	
		drive.setMaxDriveSpeed(0.5);
		drive.driveTiles(2400, 3500);
	}
	drive.killPIDs();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	bool togMOGO = 0;
	bool togWING = 0;
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	while(true)
	{	
		// ********************DRIVE********************
		// 2 stick arcade
		// double leftAxisY = MasterController.get_analog(axisRightY);
		// double rightAxisX = MasterController.get_analog(axisLeftX);
		// double leftVelocity = ((leftAxisY + rightAxisX));
		// double rightVelocity = ((leftAxisY - rightAxisX));

		// 1 stick arcade
		double leftAxisY = MasterController.get_analog(axisLeftY);
		double leftAxisX = MasterController.get_analog(axisLeftX);
		double rightAxisX = MasterController.get_analog(axisRightX);
		double aimVelocityLeft = (rightAxisX) * 0.06;
		double aimVelocityRight = -rightAxisX * 0.06;
		double leftVelocity = ((leftAxisY + leftAxisX + aimVelocityLeft));
		double rightVelocity = ((leftAxisY - leftAxisX + aimVelocityRight));

		// Tank
		// double leftAxisY = MasterController.get_analog(axisLeftY);
	    // double rightAxisY = MasterController.get_analog(axisRightY);
		// double leftVelocity = ((leftAxisY) * axisPercentBlue);
		// double rightVelocity = ((rightAxisY) * axisPercentBlue);

		
		driveLoop(leftDriveMotors, rightDriveMotors, leftVelocity, rightVelocity);

		//MOGO
		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) || MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			if(togMOGO == 1)
				togMOGO = 0;
			else
				togMOGO = 1;
			mogo.set_value(togMOGO);
		}

		//WING
		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) || MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			if(togWING == 1)
				togWING = 0;
			else
				togWING = 1;
			wing.set_value(togWING);
		}
		
		//intake / conveyor
		if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intakeMotors.move(127);
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intakeMotors.move(-127);
		}
		else
		{
			intakeMotors.brake();
		}

		//arm
		if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			arm.move_velocity(600);
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			arm.move_velocity(-600);
		}
		// else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
		// {
		// 	arm.move_velocity(-600);
		// 	pros::delay(750);
		// }
		// else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
		// {
		// 	arm.move_velocity(600);
		// 	pros::delay(750);
		// }
		else
		{
			arm.brake();
		}

		//*********************************************
	}
}