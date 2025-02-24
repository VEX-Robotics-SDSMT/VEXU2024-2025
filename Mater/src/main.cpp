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
	EncoderWheelSensorInterface encoderInterface(tracking);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	drive.setDrivePIDVals(0.9, 0, 1); //0.9, 0, 1 tuned 2/19/24
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(4, 0, 0); //4, 0, 0 tuned 2/19/24
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.8); 
	drive.setMaxTurnSpeed(0.8);

	drive.setMaxDriveAccel(0.12);

	//drive.turnDegreesAbsolute(180);
	//drive.turnDegreesAbsolute(0);

	if(skills) {
		//grab one and spin to get mogo
		intakeMotors.move(127);
		drive.driveTiles(500);
		drive.driveTiles(200);
		drive.turnDegreesAbsolute(45);
		drive.setMaxDriveSpeed(0.5);
		drive.driveTiles(1200);
		drive.driveTiles(200);
		drive.turnDegreesAbsolute(90);
		drive.setMaxDriveSpeed(0.4);
		drive.driveTiles(-600);
		mogo.set_value(1);

		//reset mogo
		drive.driveTiles(1000);

		//dump two
		arm.move(-127);
		pros::delay(750);
		arm.move(127);
		pros::delay(850);
		arm.brake();


		//dump in corner
		drive.turnDegreesAbsolute(45);
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(-1500);
		mogo.set_value(0);
		drive.driveTiles(-800, 1000);
		drive.driveTiles(300);

		//turn down middle to get next ring
		drive.turnDegreesAbsolute(0);
		drive.setMaxDriveSpeed(0.5);
		drive.driveTiles(3500);
		pros::delay(300);

		//get next MOGO
		drive.turnDegreesAbsolute(-90);
		drive.setMaxDriveSpeed(0.4);
		drive.driveTiles(-700);
		mogo.set_value(1);
		drive.setMaxDriveSpeed(0.5);
		intakeMotors.brake();
		drive.driveTiles(1300, 1500);

		//dump two
		arm.move(-127);
		pros::delay(750);
		arm.move(127);
		pros::delay(850);
		arm.brake();

		//put mogo in corner
		drive.turnDegreesAbsolute(180);
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(-1100, 1200);
		drive.turnDegreesAbsolute(160);
		mogo.set_value(0);
		drive.driveTiles(-800, 1000);
		drive.driveTiles(1200);
		drive.killPIDs();
	}

	else if(red)
	{
		//get MOGO
	drive.setMaxDriveAccel(0.5);
	drive.driveTiles(-2450);
	drive.setMaxDriveSpeed(0.2);
	drive.driveTiles(-100);
	mogo.set_value(1);

	//turn to first stack pick up one
	//intakeMotors.move(127);
	drive.setMaxDriveSpeed(0.6);
	drive.driveTiles(700);

	//regrip in case
	mogo.set_value(0);
	drive.setMaxDriveSpeed(0.2);
	drive.driveTiles(-200);
	mogo.set_value(1);

	drive.setMaxDriveSpeed(0.7);
	drive.driveTiles(500);

	//dump one
	arm.move(-127);
	pros::delay(700);
	arm.move(127);
	pros::delay(800);
	arm.brake();

	//drop MOGO and grab second
	mogo.set_value(0);
	drive.driveTiles(900);
	drive.turnDegreesAbsolute(120);
	drive.setMaxDriveSpeed(0.3);
	drive.driveTiles(-500);
	mogo.set_value(1);

	//drive to get next stack
	drive.setMaxDriveSpeed(0.6);
	intakeMotors.move(127);
	drive.driveTiles(1600);
	drive.driveTiles(200);
	//pros::delay(1200);
	drive.driveTiles(-2700); // might need to remove this

	//dump one
	arm.move(-127);
	pros::delay(700);
	arm.move(127);
	pros::delay(800);
	arm.brake();

	//drive.driveTiles(-2700);
	drive.turnDegreesAbsolute(80);

	//pick up knocked from LIGHTNING
	drive.driveTiles(600);
	drive.driveTiles(200);
	pros::delay(700);

	//dump one
	arm.move(-127);
	pros::delay(700);
	arm.move(127);
	pros::delay(800);
	arm.brake();

	//drop MOGO and get to the bar
	drive.driveTiles(-600);
	mogo.set_value(0);
	drive.turnDegreesAbsolute(-15);
	drive.setMaxDriveAccel(0.5);
	drive.driveTiles(-1700);
	drive.turnDegreesAbsolute(60);
	arm.move(-127);
	pros::delay(300);
	drive.driveTiles(-1000, 1000);

	drive.killPIDs();
	}
	else //BLUE ROUTE
	{
		
		//get MOGO
		drive.setMaxDriveAccel(0.5);
		drive.driveTiles(-2400);
		drive.setMaxDriveSpeed(0.2);
		drive.driveTiles(-100);
		mogo.set_value(1);

		//turn to first stack pick up one
		//intakeMotors.move(127);
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(700);

		//regrip in case
		mogo.set_value(0);
		drive.setMaxDriveSpeed(0.2);
		drive.driveTiles(-200);
		mogo.set_value(1);

		drive.setMaxDriveSpeed(0.6);
		drive.driveTiles(500);

		//dump one
		arm.move(-127);
		pros::delay(700);
		arm.move(127);
		pros::delay(800);
		arm.brake();

		//drop MOGO and grab second
		mogo.set_value(0);
		drive.driveTiles(900); //900
		drive.turnDegreesAbsolute(240);
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-500);
		mogo.set_value(1);

		//drive to get next stack
		drive.setMaxDriveSpeed(0.6);
		intakeMotors.move(127);
		drive.driveTiles(1650);
		drive.driveTiles(200);
		//pros::delay(1200);
		drive.driveTiles(-2250); // might need to remove this

		//dump one
		arm.move(-127);
		pros::delay(700);
		arm.move(127);
		pros::delay(800);
		arm.brake();

		//drive.driveTiles(-2700);
		drive.turnDegreesAbsolute(320);

		//pick up knocked from LIGHTNING
		drive.driveTiles(400);
		drive.driveTiles(200, 1000);
		pros::delay(700);

		//dump one
		arm.move(-127);
		pros::delay(700);
		arm.move(127);
		pros::delay(800);
		arm.brake();

		//drop MOGO and get to the bar
		drive.driveTiles(-500);
		//mogo.set_value(0);
		drive.turnDegreesAbsolute(15);
		drive.setMaxDriveAccel(0.5);
		drive.driveTiles(-1600);
		drive.turnDegreesAbsolute(300);
		arm.move(-127);
		pros::delay(300);
		drive.driveTiles(-1000, 1000);

		drive.killPIDs();
		//*/
	}	
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
	bool togLift = 0;
	bool seeBlu = false;
		bool seeRed = false; //test if can see fast
	double hue, prox;
	//arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//MasterController.clear();
	
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
		double leftVelocity = ((leftAxisY + leftAxisX));
		double rightVelocity = ((leftAxisY - leftAxisX));
		// double rightAxisX = MasterController.get_analog(axisRightX);
		// double aimVelocityLeft = (rightAxisX) * 0.06;
		// double aimVelocityRight = -rightAxisX * 0.06;
		// double leftVelocity = ((leftAxisY + leftAxisX + aimVelocityLeft));
		// double rightVelocity = ((leftAxisY - leftAxisX + aimVelocityRight));

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

		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			if(togLift == 1)
				togLift = 0;
			else
				togLift = 1;
			lift.set_value(togLift);
		}
	}
}