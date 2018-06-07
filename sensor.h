/*
 *  sensor.h
 *
 *  Sensor module: contains processes and constructor for robot sensors
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

SC_MODULE(sensor) {
	
	//INPUTS
	sc_in<bool> clock;
	sc_in<int> xpath, ypath;
	sc_in<double> speed;
	sc_in<bool> stoppedFromServer;
	sc_in<double> xO1, yO1;
	sc_in<double> xO2, yO2;
	
	//OUTPUTS
	sc_out<double> x, y;
	sc_out<double> xreq, yreq;
	sc_out<bool> stoppedFromSensor;
	
	//FIFO
	sc_fifo<int> xpathFifo, ypathFifo;
	
	//VARIABLES
	bool initPos, initTraj;
	double dt, dt_;
	int curGridx, curGridy;
	int nextGridx, nextGridy;
	double vx, vy;
	double xgridArray[12], ygridArray[10];
	
	//PROCESS: UPDATE FIFO
	void updatePathFifo() {
		xpathFifo.write(xpath.read());
		ypathFifo.write(ypath.read());
	}
	
	//PROCESS: UPDATE ROBOT TRAJECTORY
	void updateTrajectory() {
		
		//Defaults
		xreq.write(0);
		yreq.write(0);
		
		//Read Zeros
		if (initTraj && xpathFifo.num_available()>1) {
			
			//Remove Zeros
			xpathFifo.read();
			ypathFifo.read();
			initTraj = false;
			
			//Current Grid
			curGridx = xpathFifo.read();
			curGridy = ypathFifo.read();
			
			//Next Grid
			nextGridx = xpathFifo.read();
			nextGridy = ypathFifo.read();
			
			//Grid Crossing Request
			xreq.write(nextGridx);
			yreq.write(nextGridy);
			
			//Set Course
			vx = speed.read()*(nextGridx-curGridx);
			vy = speed.read()*(nextGridy-curGridy);
			
		}
		else {
			
			//bool inNextGridx = round(x.read()*100)/100 == nextGridx*2-1;
			//bool inNextGridy = round(y.read()*100)/100 == nextGridy*2-1;
			bool inNextGridx = round(x.read()*10)/10 == 0.5*(xgridArray[nextGridx-1]+xgridArray[nextGridx]);
			bool inNextGridy = round(y.read()*10)/10 == 0.5*(ygridArray[nextGridy-1]+ygridArray[nextGridy]);
			bool inNextGrid = inNextGridx && inNextGridy;
			
			if (xpathFifo.num_available()>0 && inNextGrid) {
				
				//Update Current Grid
				curGridx = nextGridx;
				curGridy = nextGridy;
				
				//Get Next Grid
				nextGridx = xpathFifo.read();
				nextGridy = ypathFifo.read();
				
				//Grid Crossing Request
				xreq.write(nextGridx);
				yreq.write(nextGridy);
				
				//Update Course
				if (xpathFifo.num_available()==0) {
					vx = 0;
					vy = 0;
				}
				else {
					vx = speed.read()*(nextGridx-curGridx);
					vy = speed.read()*(nextGridy-curGridy);
				}
			}
		}
	}
	
	//PROCESS: UPDATE ROBOT POSITION
	void updatePosition() {
		
		if (initPos && !initTraj) {

			//Assign Initial Position
			//x.write(2*curGridx-1);
			//y.write(2*curGridy-1);
			x.write(0.5*(xgridArray[curGridx-1]+xgridArray[curGridx]));
			y.write(0.5*(ygridArray[curGridy-1]+ygridArray[curGridy]));
			initPos = false;
			
		}
		else {
			if (!initPos) {
				
				//Update Position
				if (!stoppedFromServer.read() && !stoppedFromSensor.read()) {
					x.write(x.read()+vx*dt);
					y.write(y.read()+vy*dt);
				}
				else {
					x.write(x.read());
					y.write(y.read());
				}
			}
		}
	}
	
	//PROCESS: CHECK OBSTACLES
	void checkObstacles() {
		
		//Chk Obstacle-1 Distance
		double dx1 = x.read()-xO1.read();
		double dy1 = y.read()-yO1.read();
		double d1 = sqrt(dx1*dx1+dy1*dy1);
		
		//Chk Obstacle-2 Distance
		double dx2 = x.read()-xO2.read();
		double dy2 = y.read()-yO2.read();
		double d2 = sqrt(dx2*dx2+dy2*dy2);
		
		//Update 'stopped' Status
		if (d1<=1 || d2<=1) {
			stoppedFromSensor.write(true);
		}	
		else {
			stoppedFromSensor.write(false);
		}
	}
	
	//CONSTRUCTOR
	SC_HAS_PROCESS(sensor);
	sensor(sc_module_name name, double DT):sc_module(name), dt_(DT){
		
		//METHODS
		SC_METHOD(updatePathFifo);
		sensitive << xpath << ypath;
		
		SC_METHOD(updateTrajectory);
		sensitive << clock.pos();
		
		SC_METHOD(updatePosition);
		sensitive << clock.pos();
		
		SC_METHOD(checkObstacles)
		sensitive << clock.pos();
		
		//VARIABLES
		initTraj = true;
		initPos = true;
		dt = dt_;
		curGridx = 0;
		curGridy = 0;
		nextGridx = 0;
		nextGridy = 0;
		
		//X-Grid Map
		xgridArray[0] = 0;
		xgridArray[1] = 8;
		xgridArray[2] = 16;
		xgridArray[3] = 24;
		xgridArray[4] = 28;
		xgridArray[5] = 32;
		xgridArray[6] = 36;
		xgridArray[7] = 42;
		xgridArray[8] = 46;
		xgridArray[9] = 52;
		xgridArray[10] = 62;
		xgridArray[11] = 70;
		
		//Y-Grid Map
		ygridArray[0] = 0;
		ygridArray[1] = 6;
		ygridArray[2] = 12;
		ygridArray[3] = 16;
		ygridArray[4] = 22;
		ygridArray[5] = 26;
		ygridArray[6] = 32;
		ygridArray[7] = 40;
		ygridArray[8] = 46;
		ygridArray[9] = 52;
	}

};

