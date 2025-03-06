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
	EncoderWheelSensorInterface encoderInterface(driveEncoderL,driveEncoderR);
	//EncoderWheelSensorInterface encoderInterface(driveEncoderR);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	drive.setDrivePIDVals(0.75, 0, 1); //0.9 Tuned 2/1/2024
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(4.25, 0, 0); //4.25 Tuned 2/1/2024
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.75); 
	drive.setMaxTurnSpeed(0.8);
	drive.setMaxDriveAccel(0.12);

	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	
	if (skills) {
		//skills route (1 min)
		arm.move(127);
		pros::delay(500);
		arm.move(-127);
		pros::delay(700);
		arm.brake();
		//inake one slow and grab first MOGO
		intake.move(127);
		conveyorMotors.move(50);
		drive.driveTiles(1800);
		drive.turnDegreesAbsolute(270);
		intakeMotors.brake();
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-1000);
		mogo.set_value(1);

		//move arm and place ring
		arm.move(127);
		pros::delay(300);
		arm.brake();
		intakeMotors.move(127);

		//get next rings
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(-400);
		drive.turnDegreesAbsolute(180);
		drive.driveTiles(1500);
		drive.turnDegreesAbsolute(315);

		//place goal in corner
		drive.setMaxDriveSpeed(0.3);
		mogo.set_value(0);
		drive.driveTiles(-800, 1500);
		drive.driveTiles(600);

		//turn down line and grab next ring
		drive.turnDegreesAbsolute(0);
		intake.move(127);
		conveyorMotors.move(50);
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(2650);

		
		//back up to mogo
		drive.turnDegreesAbsolute(135);
		conveyorMotors.brake();
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-2500);
		mogo.set_value(1);
		intakeMotors.move(127);

		//grab rings in triangle
		drive.setMaxDriveSpeed(0.7);
		drive.turnDegreesAbsolute(100);
		drive.driveTiles(1900);
		conveyorMotors.move(-127);
		drive.driveTiles(-100);
		conveyorMotors.move(127);
		drive.turnDegreesAbsolute(0);
		wing.set_value(1);
		drive.driveTiles(1400);
		drive.turnDegreesAbsolute(90);
		conveyorMotors.move(-127);
		wing.set_value(0);
		pros::delay(200);
		conveyorMotors.move(127);
		drive.driveTiles(400);
		drive.driveTiles(-400);
		drive.turnDegreesAbsolute(270);
		drive.driveTiles(1300);
		drive.driveTiles(100);
		drive.turnDegreesAbsolute(245);

		//place goal in corner
		drive.driveTiles(-1500);
		mogo.set_value(0);
		conveyorMotors.move(-127);
		drive.setMaxDriveSpeed(0.5);
		drive.driveTiles(-800, 1000);
		drive.driveTiles(1000);
		drive.turnDegreesAbsolute(0);

		//ram wall to square back up
		drive.turnDegreesAbsolute(0);
		intakeMotors.move(127);
		drive.SetPausedPID(true);
		leftDriveMotors.move(80);
		rightDriveMotors.move(80);
		pros::delay(1000);
		leftDriveMotors.brake();
		rightDriveMotors.brake();
        intertialSensor.reset();
        while(intertialSensor.is_calibrating())
            pros::delay(50);
        drive.SetPausedPID(false);
		
		//turn along to grab next ring by wall stake
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(-300);
		drive.turnDegreesAbsolute(270);
		conveyorMotors.move(50);
		drive.driveTiles(3000);
		drive.setMaxDriveSpeed(0.5);
		drive.driveTiles(500);
		intakeMotors.brake();
		intake.move(-127);
		drive.turnDegreesAbsolute(300);
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-3200);
		mogo.set_value(1);

		//go under and score four pile
		arm.move(127);
		pros::delay(800);
		arm.brake();
		drive.setMaxDriveSpeed(0.7);
		drive.turnDegreesAbsolute(215);
		intakeMotors.move(127);
		drive.driveTiles(3000);

	}
	else {
		//MATCH AUTO RED/BLU 30sec
		//place preload on alliance stake
		arm.move(127);
		pros::delay(500);
		arm.move(-127);
		pros::delay(700);
		arm.brake();
		intakeMotors.move(127);
		pros::delay(800);
		intakeMotors.brake();
		arm.move(127);
		pros::delay(700);
		arm.brake();
		
		//back up and grab mogo
		drive.driveTiles(100);
		//drive.driveTiles(-100);
		drive.driveTiles(-1300);
		arm.move(-127);
		pros::delay(500);
		arm.brake();	
		drive.turnDegreesAbsolute(-140);
		drive.setMaxDriveSpeed(0.3);
		drive.driveTiles(-1100);
		mogo.set_value(1);
		
		//drive forward to get ring on line
		drive.setMaxDriveSpeed(.7);
		intakeMotors.move(127);
		drive.driveTiles(2080);
		pros::delay(200);

		//back up to get ring by alliance stake
		drive.driveTiles(-1800);
		conveyorMotors.move(-127);
		pros::delay(200);
		conveyorMotors.move(127);
		drive.turnDegreesAbsolute(0);
		drive.driveTiles(900);
		drive.driveTiles(100);
		drive.driveTiles(-1000);
		pros::delay(200);

		//turn to clear corner
		intakeMotors.brake();
		drive.turnDegreesAbsolute(-145);
		conveyorMotors.move(127);
		drive.driveTiles(2200);
		drive.turnDegreesAbsolute(-100);
		conveyorMotors.brake();
		wing.set_value(1);
		drive.driveTiles(700, 800);
		drive.turnDegreesAbsolute(80, 2000);

		//drop mogo incorner if red
		if(red_team) {
			mogo.set_value(0);
			conveyorMotors.move(-127);
			drive.driveTiles(-800, 1000);
			drive.driveTiles(400);
			conveyorMotors.brake();
		}

		//touch ladder with arm
		
		wing.set_value(0);
		drive.driveTiles(2600);
		arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		arm.move(127);
		pros::delay(300);
		arm.brake();

		//*/
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
	bool togARM = 0;
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

		//ARM BRAKE
		if(MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) ) {
			if(togARM == 1) {
				togARM = 0;
				arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}
				
			else {
				togARM = 1;
				arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
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
		else
		{
			arm.brake();
		}

		//*********************************************
	}
}