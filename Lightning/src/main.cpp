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
		//Start perpendicular to alliance stake intake facing away
		drive.setMaxDriveSpeed(0.5);

		//1 bring arm up and drop intake
		arm.move(127);
		pros::delay(500);
		arm.move(-127);
		intakeMotors.move(127);
		pros::delay(600);
		arm.brake();

		//drive forward pick up into arm, spin around 180
		drive.driveTiles(600);
		pros::delay(500);
		intakeMotors.brake();
		drive.turnDegreesAbsolute(178);
		pros::delay(200);

		//drive forward a tad bit and lift arm onto wall stake
		drive.driveTiles(350); //tune this a bit more
		arm.move(127);
		pros::delay(700);
		arm.brake();
		pros::delay(200);
		drive.driveTiles(-400);

		//bring arm back down drive to get MOGO on right
		// arm.move(-127);
		// drive.driveTiles(400);
		// arm.brake();
		// drive.turnDegreesAbsolute(250);	//check angle

		// //drive then turn to hit flat side of MOGO
		// drive.setMaxDriveSpeed(0.6);
		// drive.driveTiles(-2300);	//2400

		// drive.setMaxDriveSpeed(0.4);
		// drive.turnDegreesAbsolute(212); //b/t 210 and 214
		// drive.driveTiles(-1000);
		// mogo.set_value(1);

		// //turn to grap rings in line
		// drive.setMaxDriveSpeed(0.5);
		// intakeMotors.move(127);
		// pros::delay(200);
		// arm.move(90);
		// drive.turnDegreesAbsolute(0);
		// arm.brake();
		// drive.driveTiles(3200);
		// pros::delay(500);

		// //back up and turn to hit diagoanal to corner
		// drive.setMaxDriveSpeed(0.7);
		// drive.driveTiles(-1500);
		// drive.turnDegreesAbsolute(220);
		// drive.driveTiles(2200); 	//was 2000
		// drive.turnDegreesAbsolute(135);

		// //bring arm back up a bit and drive toward corner
		// drive.setMaxDriveSpeed(0.5);
		// arm.move(-90);
		// pros::delay(500);
		// arm.brake();
		// drive.driveTiles(3500, 2000);		//two second timeout
		// pros::delay(500);

		// //ram wall again for consistency
		// drive.driveTiles(-1000);
		// drive.driveTiles(1000, 1000);

		// //back up spin and drop MOGO in + corner
		// drive.driveTiles(-800);
		// drive.turnDegreesAbsolute(315);
		// intakeMotors.move(-127);
		// drive.driveTiles(-800);
		// mogo.set_value(0);
		// drive.driveTiles(800);

		// //turn drive downfield
		// drive.setMaxDriveSpeed(0.7);
		// drive.turnDegreesAbsolute(0);
		// intakeMotors.brake();
		// drive.driveTiles(4200);

		// //turn to back into second MOGO
		// drive.turnDegreesAbsolute(90);
		// drive.setMaxDriveSpeed(0.4);
		// drive.driveTiles(-1200);
		// mogo.set_value(1);
	
		// //drive forward and turn toward corner and push twice
		// drive.setMaxDriveSpeed(0.5);
		// drive.driveTiles(200);
		// intakeMotors.move(127);
		// drive.turnDegreesAbsolute(45);
		// drive.driveTiles(3000);
		// //liftintake
		// drive.driveTiles(-700);
		// drive.driveTiles(2000, 1000); //with timeout in case doesn't get there

		// //place mogo in corner
		// drive.driveTiles(-1500);
		// drive.turnDegreesAbsolute(225);
		// drive.driveTiles(-1500);
		// mogo.set_value(0);
		// drive.driveTiles(1000);
	
		//*/
	}
	else if (red_team) {
		//RED MATCH AUTO (30 sec)

		// //rush forward toward MOGO on right
		// drive.setMaxDriveAccel(0.6);
		// drive.setMaxDriveSpeed(0.8);
		
		// drive.driveTiles(-1800);
		// drive.setMaxDriveAccel(0.2);
		// drive.setMaxDriveSpeed(0.4);
		// drive.driveTiles(-150);
		// mogo.set_value(1);
		// drive.setMaxDriveSpeed(0.7);
		// drive.driveTiles(500);

		// //in case of bad grip
		// mogo.set_value(0);
		// drive.setMaxDriveSpeed(0.4);
		// pros::delay(200);
		// drive.driveTiles(-200); //400
		// mogo.set_value(1);

		
		// //drive toward alliance stake
		// drive.setMaxDriveSpeed(0.7);
		// drive.setMaxDriveAccel(0.5);
		// drive.driveTiles(1000);
		// drive.turnDegreesAbsolute(26);

		
		// // bring arm into position
		// arm.move(127);
		// pros::delay(500);
		// arm.move(-127);
		// pros::delay(700);
		// arm.brake();
		// intakeMotors.move(127);
		// pros::delay(700);
		// intakeMotors.brake();

		
		// //put preload on alliance stake
		// drive.driveTiles(980);
		// arm.move(127);
		// pros::delay(750);
		// arm.move(-100);
		// pros::delay(200);
		// arm.brake();
		
		// drive.driveTiles(-600);
		// arm.move(-127);
		// pros::delay(300);
		// arm.brake();
		// drive.turnDegreesAbsolute(-110);
		
		// //drive and pick up one
		// intakeMotors.move(127);
		// drive.setMaxDriveSpeed(0.5);
		// drive.driveTiles(1050);
		// intake.move(-127); //run intake in reverse to prevent picking up blue ring
		// //drive.driveTiles(-500);
		
		// //turn towards corner and clear it out
		// drive.turnDegreesAbsolute(-70);
		// wing.set_value(1);
		// drive.driveTiles(300);
		// intakeMotors.brake();
		
		// drive.turnDegreesAbsolute(0, 700);
		// wing.set_value(0);		
		// drive.turnDegreesAbsolute(109);
		// drive.driveTiles(200);

		// //back and drop mogo in corner
		// mogo.set_value(0);
		// drive.driveTiles(-800, 800);

		// //go touch bar for WP
		// drive.setMaxDriveSpeed(0.7);
		// drive.driveTiles(1800, 2000);
		// drive.killPIDs();
		// arm.move(127);
		// pros::delay(100);
		// arm.brake();
		

		arm.move(127);
		pros::delay(500);
		arm.move(-127);
		pros::delay(700);
		arm.brake();
		intakeMotors.move(127);
		pros::delay(800);
		intakeMotors.brake();
		arm.move(127);
		pros::delay(900);
		arm.brake();
		
		drive.driveTiles(-600);
		arm.move(-127);
		pros::delay(900);
		arm.brake();
		drive.driveTiles(-1500);
		drive.turnDegreesAbsolute(90);
		drive.setMaxDriveSpeed(0.5);
		drive.driveTiles(2200, 2500);
		
		arm.move(100);
		pros::delay(250);
		arm.brake();

		//*/
	}
	else {
		//BLUE MATCH AUTO (30 sec)

		//rush forward toward MOGO on right
		drive.setMaxDriveAccel(0.6);
		drive.setMaxDriveSpeed(0.8);
		
		drive.driveTiles(-3250);
		drive.setMaxDriveAccel(0.2);
		drive.setMaxDriveSpeed(0.4);
		drive.driveTiles(-150);
		mogo.set_value(1);
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(800);

		//in case of bad grip
		mogo.set_value(0);
		drive.setMaxDriveSpeed(0.4);
		drive.driveTiles(-400); //400
		mogo.set_value(1);

		
		//drive toward alliance stake
		drive.setMaxDriveSpeed(0.7);
		drive.setMaxDriveAccel(0.5);
		drive.driveTiles(2200);
		drive.turnDegreesAbsolute(-12);
		
		// bring arm into position
		arm.move(127);
		pros::delay(500);
		arm.move(-127);
		pros::delay(700);
		arm.brake();
		intakeMotors.move(127);
		pros::delay(700);
		intakeMotors.brake();

		//put preload on alliance stake
		intake.move(127);
		drive.driveTiles(1200);
		arm.move(127);
		pros::delay(770);
		arm.move(-100);
		pros::delay(100);
		arm.brake();
		drive.driveTiles(-2000);
		
		
		arm.move(-127);
		pros::delay(300);
		arm.brake();

		drive.turnDegreesAbsolute(270);
		drive.driveTiles(1500, 2000);
		drive.killPIDs();
		/*
		drive.turnDegreesAbsolute(117);
		//drive and pick up one
		intakeMotors.move(127);
		drive.setMaxDriveSpeed(0.4);
		drive.driveTiles(1700);
		intake.move(-127);
		pros::delay(1000);
		drive.driveTiles(-450);

		//turn towards corner and clear it out
		drive.turnDegreesAbsolute(74);
		intakeMotors.brake();
		wing.set_value(1);
		drive.driveTiles(800);
		drive.turnDegreesAbsolute(251);
		intakeMotors.brake();

		//go touch bar for WP
		wing.set_value(0);
		drive.driveTiles(300);
		drive.driveTiles(-800);
		mogo.set_value(0);
		drive.driveTiles(-500, 1000);
		drive.setMaxDriveSpeed(0.7);
		drive.driveTiles(3700, 2200);
		drive.killPIDs();
		arm.move(-127);
		pros::delay(200);
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
	bool togWING = 0;
	//arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	while(true)
	{	
		// ********************DRIVE********************
		// 2 stick arcade
		double leftAxisY = MasterController.get_analog(axisRightY);
		double rightAxisX = MasterController.get_analog(axisLeftX);
		double leftVelocity = ((leftAxisY + rightAxisX));
		double rightVelocity = ((leftAxisY - rightAxisX));

		// 1 stick arcade
		// double leftAxisY = MasterController.get_analog(axisLeftY);
		// double leftAxisX = MasterController.get_analog(axisLeftX);
		// double rightAxisX = MasterController.get_analog(axisRightX);
		// double aimVelocityLeft = (rightAxisX) * 0.06;
		// double aimVelocityRight = -rightAxisX * 0.06;
		// double leftVelocity = ((leftAxisY + leftAxisX + aimVelocityLeft));
		// double rightVelocity = ((leftAxisY - leftAxisX + aimVelocityRight));

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