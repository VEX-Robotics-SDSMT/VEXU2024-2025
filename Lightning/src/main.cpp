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
void disabled() {}

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
	
	EncoderWheelSensorInterface encoderInterface(driveEncoderL,driveEncoderR);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	drive.setDrivePIDVals(0.75, 0, 1); // 0.75, 0, 1 tuned 1/25/2024
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(4.25, 0, 0); //4.25, 0, 0 tuned 1/25/2024
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.75); 
	drive.setMaxTurnSpeed(0.8);

	drive.setMaxDriveAccel(0.12);

	//drive.turnDegreesAbsolute(180);
	//drive.turnDegreesAbsolute(0);

	drive.driveTiles(1000);
	//drive.driveTiles(-1000);
	
	
	//*/
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
	bool togLED = 0;
	bool togColor = 0; //0 for red, 1 for blu
	bool seeBlu = false;
		bool seeRed = false; //test if can see fast
	double hue, prox;
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	colorSensor.disable_gesture();
	MasterController.clear();
	
	while(true)
	{	
		// ********************DRIVE********************
		// 2 stick arcade
		double leftAxisY = MasterController.get_analog(axisLeftY);
		double rightAxisX = MasterController.get_analog(axisRightX);
		double leftVelocity = ((leftAxisY + rightAxisX));
		double rightVelocity = ((leftAxisY - rightAxisX));

		// 1 stick arcade
		//double leftAxisY = MasterController.get_analog(axisLeftY);
		//double leftAxisX = MasterController.get_analog(axisLeftX);
		//double rightAxisX = MasterController.get_analog(axisRightX);
		//double aimVelocityLeft = (rightAxisX) * 0.06;
		//double aimVelocityRight = -rightAxisX * 0.06;
		//double leftVelocity = ((leftAxisY + leftAxisX + aimVelocityLeft));
		//double rightVelocity = ((leftAxisY - leftAxisX + aimVelocityRight));

		// Tank
		// double leftAxisY = MasterController.get_analog(axisLeftY);
	    // double rightAxisY = MasterController.get_analog(axisRightY);
		// double leftVelocity = ((leftAxisY) * axisPercentBlue);
		// double rightVelocity = ((-rightAxisY) * axisPercentBlue);

		
		driveLoop(leftDriveMotors, rightDriveMotors, leftVelocity, rightVelocity);

		//toggle MOGO
		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			if(togMOGO == 1)
				togMOGO = 0;
			else
				togMOGO = 1;
			mogo.set_value(togMOGO);
		}
		
		//intake / conveyor
		if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake.move(-127);
			conveyorMotors.moveVelocity(-600);
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake.move(127);
			conveyorMotors.moveVelocity(600);
		}
		else
		{
			intake.brake();
			conveyorMotors.brake();
		}

		//manual arm
		if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			arm.move_velocity(600);
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			arm.move_velocity(-600);
		}
		else
		{
			arm.brake();
		}

		//auto arm
		if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			arm.move_velocity(600);
			pros::delay(500);
			arm.brake();
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			arm.move_velocity(-600);
			pros::delay(500);
			arm.brake();
		}

		//toggle LED for sensor
		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			seeBlu = false;
			seeRed = false;
			if(togLED)
				togLED = 0;
			else
				togLED = 1;
			if(togLED)
				colorSensor.set_led_pwm(100);
			else
				colorSensor.set_led_pwm(0);
		}

		//toggle color we are looking for
		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			if(togColor)
				togColor = 0;
			else
				togColor = 1;
		}

		//*****color sensing funtions and testing******
		hue = colorSensor.get_hue();
		
		//prox = colorSensor.get_proximity();
		MasterController.print(0, 0, "%f", hue);
		hue = colorSensor.get_hue();
		if((hue > 0 && hue < 10) || (hue > 340 && hue < 360)) {
			MasterController.print(0, 11, "%s", "R");
			seeRed = true;
		}
		else if(hue > 160 && hue < 190) {
			MasterController.print(0, 11, "%s", "B");
			seeBlu = true;
		}

		if(seeRed)
			MasterController.print(0, 13, "%s", "SR");
		if(seeBlu)
			MasterController.print(0, 16, "%s", "SB");
		if(togColor)
			MasterController.print(1, 0, "%s", "Target: Blue");
		else
			MasterController.print(1, 0, "%s", "Target: Red");

		
		MasterController.clear();



		


		//*********************************************
	}
}