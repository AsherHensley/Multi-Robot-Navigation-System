/*
 *  obstacle.h
 *
 *  Obstacle module: contains processes and constructor for robot obstacles
 *
 *  Created by Asher Hensley on 11/3/16.
 *  Copyright 2016. All rights reserved.
 *
 *  MIT License
 *
 *  Copyright (c) 2016 Asher A. Hensley Research
 *  $Revision: 1.0 $  $Date: 2016/11/03 19:23:54 $
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a 
 *  copy of this software and associated documentation files (the 
 *  "Software"), to deal in the Software without restriction, including 
 *  without limitation the rights to use, copy, modify, merge, publish, 
 *  distribute, sublicense, and/or sell copies of the Software, and to 
 *  permit persons to whom the Software is furnished to do so, subject to 
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included 
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "systemc.h"

SC_MODULE(obstacle) {
	
	//VARIABLES
	double dt, dt_;		// clock period (in seconds)
	double xi, xi_;		// position in x (meters)
	double yi, yi_;		// position in y (meters)
	double vx, vx_;		// velocity in x (meters/second)
	double vy, vy_;		// velocity in y (meters/second)
	double dx, dx_;		// change in x before turning
	double dy, dy_;		// change in y before turning
	bool init;		// initialization Flag
	double xclks;
	double yclks;
	bool dirstate;		// Direction state (0 is x, 1 is y)
	int k;

	//INPUTS
	sc_in<bool> clock;
	
	//OUTPUTS
	sc_out<double> x;	// x-position (meters) 
	sc_out<double> y;	// y-position (meters)
	
	//PROCESSES
	void sensorPositionUpdate() {
		if (init) {
			x.write(xi);
			y.write(yi);
			init = false;
		}
		else {
			
			if (dirstate==0) {
				if (k<xclks) {
					k++;
				}
				else {
					dirstate = true;
					k = 1;
					vy = -vx;
					vx = 0;
				}
			}
			else {
				if (k<yclks) {
					k++;
				}
				else {
					dirstate = false;
					k = 1;
					vx = vy;
					vy = 0;
				}
			}
			
			x.write(x.read()+vx*dt);
			y.write(y.read()+vy*dt);

		}
	}

	//CONSTRUCTOR
	SC_HAS_PROCESS(obstacle);
	obstacle(sc_module_name name,double XI,double YI,double VX,double VY,double DT,double DX,double DY):
	sc_module(name),xi_(XI),yi_(YI),vx_(VX),vy_(VY),dt_(DT),dx_(DX),dy_(DY){
	
		//METHODS
		SC_METHOD(sensorPositionUpdate)
		sensitive << clock.pos();
		
		//VARIABLES
		dt = dt_;			// clock period (in seconds)
		xi = xi_;			// position in x (meters)
		yi = yi_;			// position in y (meters)
		vx = vx_;			// velocity in x (meters/second)
		vy = vy_;			// velocity in y (meters/second)
		dx = dx_;			// change in x (meters)
		dy = dy_;			// change in y (meters)
		init = true;		// Initialization Flag
		dirstate = false;	// Direction state (0 is x, 1 is y)
		k = 1;
		
		//TURN CLOCKS
		double speed = abs(vx+vy); //assume one is zero
		xclks = dx/(speed*dt);
		yclks = dy/(speed*dt);
		
	}
};

