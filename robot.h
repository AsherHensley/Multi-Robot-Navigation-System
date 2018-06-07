/*
 *  robot.h
 *
 *  Robot module: contains processes and constructor for robot instances
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

SC_MODULE(robot) {

	
	//INPUTS
	sc_in<bool> clock;
	sc_in<double> xreqIn, yreqIn;
	sc_in<int> okToCross;
	sc_in<bool> stoppedFromSensor;
	
	//OUTPUT
	sc_out<double> xreqOut, yreqOut;
	sc_out<bool> stoppedFromServer;
	sc_out<int> obstacleDetected;
	
	//PROCESS: SEND GRID CROSSING REQUEST TO SERVER
	void relayGridRequest() {
		if (xreqIn.read()>0 && yreqIn.read()>0) {
			xreqOut.write(xreqIn.read());
			yreqOut.write(yreqIn.read());
		}
		else {
			if (okToCross.read()==1) {
				xreqOut.write(0);
				yreqOut.write(0);
			}
		}
	}
	
	//PROCESS: PROCESS SERVER REPLIES
	void procServerPermissions() {
		if (okToCross==2) {
			stoppedFromServer.write(1);
		}
		else {
			if (okToCross==1) {
				stoppedFromServer.write(0);
			}
		}

	}
	
	//PROCESS: PROCESS OBSTACLE ALERT FROM SENSOR
	void procObstacleAlert() {

		if (stoppedFromSensor.read()==1) {
			
			//Tell Server Robot Has Stopped
			obstacleDetected.write(1);
		}
		else {
			if (obstacleDetected.read()==1 && stoppedFromSensor.read()==0) {
				
				//Tell Server Robot Has Resumed
				obstacleDetected.write(2);
			}
			else {
				obstacleDetected.write(0);
			}

		}
	}
	
	//CONSTRUCTOR
	SC_HAS_PROCESS(robot);
	robot(sc_module_name name):sc_module(name){
		
		//METHODS
		SC_METHOD(relayGridRequest);
		sensitive << clock.pos();
		
		SC_METHOD(procServerPermissions);
		sensitive << okToCross;
		
		SC_METHOD(procObstacleAlert);
		sensitive << clock.pos();
		
	}
};



