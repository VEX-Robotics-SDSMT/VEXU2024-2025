#include "main.h"
#include "DiffDrive.h"
#include "botFunctions.h"
#include "globals.h"
#include "pros/rtos.h"

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
	intertialSensor.reset();
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
	/*
	EncoderWheelSensorInterface encoderInterface(driveEncoder);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	drive.setDrivePIDVals(2, 0, 0);
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(3.5, 0, 5); //tuned 10/19/2024
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(1); 
	drive.setMaxTurnSpeed(0.8); //tuned 10/19/2024

	drive.setMaxDriveAccel(0.12);

	//drive.turnDegreesAbsolute(180);
	//drive.turnDegreesAbsolute(0);

	drive.driveTiles(1000);
	//*/
	mogo.set_value(0);
	rightDriveMotors.move(-100);
	leftDriveMotors.move(-100);
	pros::delay(700);
	rightDriveMotors.brake();
	leftDriveMotors.brake();
	mogo.set_value(1);
	intake.move(-127);
	conveyorMotors.move(-127);
	rightDriveMotors.move(60);
	leftDriveMotors.move(60);
	pros::delay(800);
	rightDriveMotors.move(20);
	leftDriveMotors.move(20);
	pros::delay(1000);
	rightDriveMotors.brake();
	leftDriveMotors.brake();
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
	while(true)
	{	
		// ********************DRIVE********************
		// 2 stick arcade
		//double leftAxisY = MasterController.get_analog(axisLeftY);
		//double rightAxisX = MasterController.get_analog(axisRightX);
		//double leftVelocity = ((leftAxisY + rightAxisX));
		//double rightVelocity = ((leftAxisY - rightAxisX));

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
		// double rightVelocity = ((-rightAxisY) * axisPercentBlue);

		
		driveLoop(leftDriveMotors, rightDriveMotors, leftVelocity, rightVelocity);


		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
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

		//lifting
		if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_X))
		{
			liftMotors.moveVelocity(600);
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			liftMotors.moveVelocity(-600);
		}
		else
		{
			liftMotors.brake();
		}

		//*********************************************
	}
}
