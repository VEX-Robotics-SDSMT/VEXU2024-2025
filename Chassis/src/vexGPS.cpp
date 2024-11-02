#include "vexGPS.h"

//write excess functions here for driving with GPS

/*
gpsdrive
distance - the distance in meters that you want to drive
xdir is +1 if going in positive x direction, -1 otherwise
ydir is +1 if going in positive y direction, -1 otherwise
speed is the speed to run the motors, negative if traveling backwards

*/ 
void gpsdrive(double distance, int xdir, int ydir, double speed, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps)
{
    MasterController.print(0, 0, "%s", "enter gps drive");
    pros::delay(200);
    MasterController.clear_line(0);
    pros::c::gps_status_s_t stat = gps.get_status();
	double currx = stat.x;
	double curry = stat.y;
    //MasterController.print(0, 0, "%f", currx);
    //MasterController.print(0, 20, "%f", curry);
    double angle = gps.get_heading(); //get degrees
    //MasterController.print(0, 0, "%f", angle);
    double trigAngle = angle;
    while (trigAngle > 90)
        trigAngle -= 90;
    double addx = distance * cos(trigAngle); //distance to add to currx and y
    double addy = distance * sin(trigAngle);

    double newx = currx + xdir*(addx); //newx,y are coordinates to travel to
    double newy = curry + ydir*(addy);
    //MasterController.print(0, 0, "%f", newx);
    //MasterController.print(0, 20, "%f", newy);
    bool go = true;

    //for now will test with an error tolerance of 100mm
    //MasterController.clear_line(0);
    //MasterController.print(0, 0, "%s", "Loop");
    while(!((newx - 0.1) < currx < (newx + 0.1)) && !((newy - 0.1) < curry < (newy + 0.1)) && go)
    {
        //MasterController.clear_line(0);
        //MasterController.print(0, 0, "%s", "Entered Loop");
        right.move(speed), left.move(speed);
        stat = gps.get_status();
        currx = stat.x, curry = stat.y;
        //MasterController.print(0, 0, "%f", currx);
        //MasterController.print(0, 20, "%f", curry);
        if(((newx - 0.1) < currx < (newx + 0.1)) && ((newy - 0.1) < curry < (newy + 0.1)))
        {
            go = false;
            right.brake();
            left.brake();
        }
        //pros::delay(20);
    }

    return;
} 

void gpsturn(double tgt, double speed, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps)
{
    double tol = 30.0; //tolerance on turning
    double angle = gps.get_heading();
    bool go = true;

    //should find an algorithm to determine clock vs counter
    //higher speed requires higher tolerance to hit tgt than lower speeds
    //counter-clockwise
    while(((tgt) < angle < (tgt + tol)) && go)
    {
        right.move(speed), left.move(-speed);
        angle = gps.get_heading();
        MasterController.clear_line(0);
        MasterController.print(0, 0, "%f", angle);
        if(((tgt) < angle) && (angle < (tgt + tol)))
            right.brake(), left.brake(), go = false;
    }
    MasterController.clear_line(0);
    MasterController.print(0, 0, "%f", angle);
    right.brake(), left.brake();
    
    return;
}