#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include <iomanip>   // I/O manipulators

#include <windows.h> // for keyboard input

// user defined functions
#include "timer.h" // for measuring time
#include "rotation.h" // for computing rotation functions
#include "3D_graphics.h" // user functions for DirectX 3D graphics

#include "graphics.h"

#include "robot.h"

#include "controller.h"

#include "curve.h"

using namespace std;

double speed_PID(double wb , double w_des , double dt);  // maitaining the speed 
double slip_controller( double dt  , double sr , double w_des); // accelerating (traction)
double brake_controller( double dt  , double sr , double w_des);// deaccelerating (brake)


const double PI = 4*atan(1.0);

// output file for testing / debugging purposes since 
// console output is not easy in a graphics program
ofstream fout("output.txt");

/// Start the car from the beginning of the track ***************************
// initial location of car
// -> you can start the car on any spot of the track
double X0 = -80.1345;
double Y0 = 11.2148;
double THETA_0 = 0.0;

extern robot robot1;

// make global since u_phi is needed by the graphics program
// to draw the angle of the front wheels when they steer
double u_s = 0.0, u_phi = 0.0;	

/// global varriables for other controllers' implementation ******
const double wmax = 810.0; // maximum back wheel speed (rad/s)	
const double V_bat = 12.0; // lipo battery voltage
const double V_bat_inv = 1/V_bat;
const double wmax_inv = 1/810.0 ; 	
const double Rw = 3.2e-2; // (m) 3.2 cm tire radius
const double tol = 0.001;
/// *********

void reset_ICs();

// note that the following views are availble in the simulation
// by pressing the number keys

// 1 - overhead view (3rd person perspective)
// 2 - driver view (1st person perspective)
// 3 - diagonal view (3rd person perspective)
// 4 - close up overhead driver view (1st person perspective)
// 5 - overhead driver view (1st person perspective)

// note calculate_control_inputs() is analogous to loop()
// in an Arduino program -- it get's exectuted as fast as possible.
// -- ie this isn't like a main

void calculate_control_inputs()
{	
	// set state variables and outputs each time in the control loop
	
	double im = robot1.x[1]; // motor current im (A)
	double wm = robot1.x[2]; // motor speed wm (rad/s)
	double theta = robot1.x[3]; // motor angle, theta (rad)	
	double vf = robot1.x[4]; // forward velocity vf (m/s)
	double xf = robot1.x[5]; // forward position xf (m)
	double theta_c = robot1.x[6]; // robot angle theta_c (rad)
	double xc = robot1.x[7]; // robot x position xc (m)
	double yc = robot1.x[8]; // robot y position yc (m)	

	double wb = robot1.y[1]; // back wheel velocity wb (rad/s)
	double wf = robot1.y[2]; // front wheel velocity wf (rad/s)
	double r = robot1.y[3]; // slip ratio r
	double mu = robot1.y[4]; // coefficient of tire friction mu

	double t = robot1.t; // simulation time

	// max steering angle of typical car 
	double phi_max = 35.0/180*3.14159; // 0.61 rad
	
	// maximum car voltage (speed input) (V)
	double us_max = 12; // 12 V -> 26.3 m/s -> 95 hm/h, 60 mph !	
	
	double ds, dphi;

//	static double t_reset = 30;

	static int init = 0;
	
	double pixels_per_m = 21.15; // scale of track images
	
	// coordinate offset between half track and full track coord
	double x_offset = -1690.5; 
	
	double s_begin = 0.0; // begining of curve
	double s_end = 26900.0; // end of curve
	double s,x,y,xd,yd,xdd,ydd;
		

	/// Put the coordinates of all the points on the track into two arrays ********
	int i; // counter, used in for-loops.
	double x_tr[26901], y_tr[26901]; // arrays containing coordinates of the track points
	static double tp = 0;  // previous time
	static double ei = 0;  // integral of the error
	static double ep = 0; // previous error

	static double yc_p = X0; // y position of the car in the previous iteration
	static double xc_p = Y0; // x position of the car in the previous iteration

	// The angle between the car and the road in the previous iteration
	static double angle_car_road_p = 0;
	
	
	// initialization section -- gets exectuted once at the beginning
	if( !init ) {
			
		for (i=0;i<=26900;i++){
			curve(i, x, y, xd, yd, xdd, ydd);
		
			// adjust x offset between full and half track
			x = x + x_offset;
		
			// convert x and y to m
			x = x / pixels_per_m;
			y = y / pixels_per_m;
//			theta = atan2(yd,xd);

			x_tr[i]=x;
			y_tr[i]=y;
			
//			fout << x << " , " << y << "\n"; // to print the initial location of the track
			// when i==0
			
		}
		
//		fout << "%xc" << " , " << "yc" << "\n";
		fout << "%t" << " , " << "vf"  << " , " << "sr" << " , " << "u_s" << "\n";
	/// **************************	
	
		fout << scientific;

		init = 1;
	}
	
	// Check to see if time has passed, then calculate the control signal
	if ( t - tp != 0 ) {
	// read inputs from the keyboard ////////////
	
	
/* 	ds = 0.01; // V
	dphi = 0.003; // rad
	
	if( KEY(VK_UP) ) {
		if( u_s < us_max ) u_s += ds;		
	}
		
	if( KEY(VK_DOWN) ) {
		if( u_s > -us_max ) u_s -= ds;
	} */
		
/* 	if( KEY(VK_LEFT) ) {
		if( u_phi < phi_max ) u_phi += dphi;
	}

	if( KEY(VK_RIGHT) )	{
		if( u_phi > -phi_max ) u_phi -= dphi;	
	} */
	
	// set car position and angle around the track
	// -- note this isn't a valid control scheme
//	s = 500*t; // increase to start along the track curve a distance of s
	
	
	// calculate the x, y position (pixels) and derivatives 
	// of the center of the full car track
	// as a function of s the distance along the curve
	// from the beginning.
	// note that s is valid from s_begin to s_end
	// (plus a little bit before and after that if needed)
/* 	curve(s,x,y,xd,yd,xdd,ydd);	
		
	// adjust x offset between full and half track
	x = x + x_offset;
		
	// convert x and y to m
	x = x / pixels_per_m;
	y = y / pixels_per_m;
	theta = atan2(yd,xd); */
		
		
	/// Find the closest distance from the track to the car ********************
	double distance; // Distance between the car and any point on the track
	double min_distance = 20000; // Distance between the car and the closest point on the track
	double x_front, y_front;	//coordinates for a point on the track that is a 
	// short distance in front of the closest point on the track to the car
	double x_closest, y_closest; // Coordinates of the closest point on the track to the car
		
	for (i=0;i<=26900;i++){
		
		// formula for the distance between two points
		distance = sqrt(((xc - x_tr[i]) * (xc - x_tr[i])) +
			((yc - y_tr[i]) * (yc - y_tr[i])));
			
			if (distance<min_distance) {
				
				min_distance = distance;
				
				x_closest = x_tr[i];
				y_closest = y_tr[i];
				
				// Calculates the coordinates for a point on the track that is a 
				// short distance in front of the closest point on the track to the car	
				x_front = x_tr[i + 1];
				y_front = y_tr[i + 1];
			
			}
	}
	
	/// Find out on which side of the road the car is ***********************
	double road_side = 0;
	double position = 0;
	
	// sign of the determinant of vectors AB and AM, 
	// where M(xc, yc) is the location of the car. 
	// A(x_closest, y_closest) and B(x_front, y_front)
	position = (x_front - x_closest) * (yc - y_closest) - (y_front - y_closest) * (xc - x_closest);
	
	if (position > 0) {
		road_side = -1; // car is on the left side of the road
	} else if (position < 0) {		
		road_side = 1; // car is on the right side of the road
	} else if (position == 0) {
		road_side = 0; // car is on the road
	}
	/// ***********************************************************************


	/// Calculate the angle between the road and the car *******************
	double y_car, x_car;
	double y_road, x_road;
	double road_tan[2], car_tan[2];
	double dot_product;
	double road_tangent_mag, car_tangent_mag;
	double cos_angle, angle_car_road = 0;

	// Vector tangent to the road
	y_road = y_front - y_closest;
	x_road = x_front - x_closest;

	// car's displacement in one iteration
	// Coordinates of the vector tangent to the car's direction of movement
	y_car = yc - yc_p;
	x_car = xc - xc_p;

	// dot product of the above vectors
	dot_product = x_road * x_car + y_road * y_car;	
	
	// Magnitude of the vectors
	road_tangent_mag = (sqrt((x_road * x_road) + (y_road * y_road)));
	car_tangent_mag = (sqrt((x_car * x_car) + (y_car * y_car)));
	
	// Calculate the angle between the two vectors
	if (vf != 0) { // to avoid division by zero, we assume the condition of having speed
	
	// Cosine of the angle between the two vectors
	cos_angle = dot_product / (road_tangent_mag * car_tangent_mag);
	
	// The angle between the car and the road (rad)
	angle_car_road = acos (cos_angle); 
	}
	/// **********************************
	 
	 
	/// PID controller ***************
	double kp; // proportional gain
	double ki; // integral gain
	double kd; // derivative gain
	double e; // Error
	double ed; // derivative of the error
	double dt; // Timestep
	double u_phi_abs; // Absolute value of error
	dt = (t - tp); // period between two iterations
	
	e = min_distance;
	ei += e * dt; // Reiman sum approximation
	ed = (e - ep) / dt;
	
	kp = 0.08;
	kd = 0.01;
	ki = 0.001;
	
	//PID controller formula
	u_phi_abs = kp * e + ki * ei + kd * ed;
		
	// Saturate steering angle input
	if (u_phi_abs > (phi_max)) u_phi_abs = (phi_max);
	
	// Final steering angle
	u_phi = (road_side * u_phi_abs);
	
	///********************************
	
	
	/// implementation of other controllers ****************
	
	// How much should braking decrease the speed in each iteration
	double vf_b;
	vf_b = 0.02;
	
	const double vf_des = 26; // car's desired speed (set to maximum speed)
	double w_des;  // car wheels desired angular speed
	double sr; // instant slip ratio of back wheels
	
	w_des = vf_des / Rw;
	
	sr = ((wb * Rw)- vf)/ (abs(vf)+tol);

	// Switching between brake, speed, and traction controllers
	if( KEY(0x42) ){ // Brake when B is pressed on the keyboard
	
		if (vf > 0){ // Stop the car from going backwards
			// brake
			u_s = brake_controller( dt  , sr , vf - vf_b);
		} else u_s = 0;
		
	} else {
		
		if (vf < vf_des - (0.1*vf_des)) { // if the speed is too low
			// traction;
			u_s = slip_controller( dt  , sr , w_des);
			
		} else if (angle_car_road > 0.2) { // if the car is facing a sharp turn
		
			// brake
			u_s = brake_controller( dt  , sr , vf - vf_b);
			
		} else { // maintain the desired speed is not reached
		
			// speed control
			u_s = speed_PID( wb , w_des , dt);
		}
	}
	
	 
//	u_s = 12;
	
	/// TEST *********************
	 
//	u_s = speed_PID( wb , w_des , dt);


/* 	
	if( t>10 ){ // Brake when B is pressed on the keyboard
	
		if (vf > 0){ // Stop the car from going backwards
			// brake
			u_s = brake_controller( dt  , sr , vf - vf_b);
		} else u_s = 0;
		
	} else u_s = 12;
	 */
	
	
//	u_s = slip_controller( dt  , sr , w_des);
	
	/// *********************
	
	
	// Saturate voltage input
	if (u_s > us_max) {
		u_s = us_max;
	} else if (u_s < -us_max) {
		u_s = -us_max;
	}
	
	/// *****************
	
	
	
	fout << t << " , " << vf << " , " << sr << " , " << u_s << "\n";
//	fout << xc << " , " << yc << "\n";

	/// set the previous values for the next iteration ********
	ep = e;
	tp = t;

	yc_p = yc;
	xc_p = xc;
	
	angle_car_road_p = angle_car_road;

		
	// start track on the curve
	/* robot1.x[7] = x;
	robot1.x[8] = y;		
	robot1.x[6] = theta; */

	//////////////////

	// set inputs in the robot model
	robot1.u[1] = u_s; // motor voltage V(t)
	robot1.u[2] = 0.0; // disturbance torque Td(t)
	robot1.u[3] = u_phi; // steering angle phi (rad)
	
	// file output
	// note: too much output might slow down the controller and simulation

}
	// how to periodically reset the ICs
	// -- in case you want to perform some repeated tests, etc.
//	if( t > t_reset ) {
//		reset_ICs();
//		t_reset += 30; // reset 30 seconds later
//	}

}


void reset_ICs()
{
	// reset inputs
	
	u_s = 0.0;
	u_phi = 0.0;		
	
	robot1.u[1] = 0.0; // motor voltage V(t)
	robot1.u[2] = 0.0; // disturbance torque Td(t)
	robot1.u[3] = 0.0; // steering angle phi (rad)
	
	// reset states
	
	robot1.x[1] = 0.0; // motor current im (A)
	robot1.x[2] = 0.0; // motor speed wm (rad/s)
	robot1.x[3] = 0.0; // motor angle, theta (rad)	
	robot1.x[4] = 0.0; // forward velocity vf (m/s)
	robot1.x[5] = 0.0; // forward position xf (m)
	robot1.x[6] = THETA_0; // robot angle theta_c (rad)
	robot1.x[7] = X0; // robot x position xc (m)
	robot1.x[8] = Y0; // robot y position yc (m)	
	
	// reset outputs
	
	robot1.y[1] = 0.0; // back wheel velocity wb (rad/s)
	robot1.y[2] = 0.0; // front wheel velocity wf (rad/s)
	robot1.y[3] = 0.0; // slip ratio r
	robot1.y[4] = 0.0; // coefficient of tire friction mu
	
}




double speed_PID(double wb , double w_des , double dt)

{
	double e_s;
	double u_s;
	double de_s = 0.0;
	double ie_s = 0.0;
	double voltage_s;
	double ep = 0.0;


	double ki_s = 150; //kd is considered zero to avoid set point kick phenomenom. Therefor,
                      // we decided to use a PI controller. 
	double kp_s = 200;
	double kd_s = 0;
	
	e_s = w_des - wb ; // error signal 
	 
	
	
	
	ie_s += e_s * dt; // integration of error
	
	u_s = kp_s * e_s + kd_s * de_s + ki_s * ie_s; // actuator signal 
	
	
	
	voltage_s = u_s * wmax_inv * V_bat ; // connvert to battry voltage. 
	
	
	if( voltage_s > V_bat ) voltage_s = V_bat;    // saturation of actuator signal 
	if( voltage_s < -V_bat )voltage_s = -V_bat;
	
	return voltage_s;
	
}

double slip_controller( double dt  , double sr , double w_des)


{
	double wb;
	
	double v_des;
	
	double e;
	double ei = 0.0 ;
	double u_t;
	double voltage_t;
	double sr_max = 0.1; // in BW model the maximum friction coefficient (mu_max = 0.81)
	                    // is corresponded to (slip ratio = 0.1)
	
//	double kp = 5.0;
//	double ki = 10.0;

	double kp = 50;
	double ki = 10.0;

	
	v_des = w_des * Rw;
	
	e = sr_max - sr ; 
	
	ei += e * dt;
	
	u_t = kp * e + ki * ei ;
	
	
	wb = (v_des + u_t * (abs(v_des) + tol))/ Rw ; // calculating wb with respect to desired 
	                                         // slip ratio.
		
	
	voltage_t = wb * wmax_inv * V_bat ;  // convert to battery voltage 
	
	
	if( voltage_t > V_bat ) voltage_t = V_bat;  // saturation of actuator signal 
	if( voltage_t < -V_bat )voltage_t = -V_bat;
	
	return voltage_t;
	
	
}


double brake_controller( double dt , double sr , double w_des)


{
	double wb;
	double v_des; 
	
	double e ; 
	double ei = 0.0;
	double u_b;
	
	double voltage_br;
	double sr_min = -0.1; // in BW model the maximum friction coefficient (mu_max = -0.81)
	                     // is corresponded to (slip ratio = -0.1)
	
	float kp = 2500.0;
	
	float ki = 100.0;
	
	v_des = w_des * Rw;
	
	e = sr_min - sr ; 
	
	ei += e * dt ; 
	
	u_b = kp * e + ki * ei ;
	
	
	
	wb = (v_des + u_b * (abs(v_des) + tol))/ Rw ; // calculating wb with respect to desired 
	                                         // slip ratio.
	
	
	
	voltage_br = wb * wmax_inv * V_bat ; // conversion of actuator signal to battery voltage  
	
	
	if( voltage_br > V_bat ) voltage_br = V_bat;  // saturation of actuator signal
	if( voltage_br < -V_bat )voltage_br = -V_bat;
	
	return voltage_br;
	
	
}
