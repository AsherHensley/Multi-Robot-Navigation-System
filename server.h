/*
 *  server.h
 *
 *  Server module: contains processes and constructor for server
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

SC_MODULE(server) {
	
	//INPUTS
	sc_in<bool> clock;
	sc_in<double> xreq1, yreq1;
	sc_in<double> xreq2, yreq2;
	sc_in<double> xreq3, yreq3;
	sc_in<double> x1, y1, x2, y2, x3, y3;
	
	//OUTPUTS
	sc_out<int> xpath1, ypath1, okToCross1;
	sc_out<int> xpath2, ypath2, okToCross2;
	sc_out<int> xpath3, ypath3, okToCross3;
	sc_out<double> speed1, speed2, speed3;
	
	//VARIABLES
	int xpathArray1[13], ypathArray1[13], ptr1, tentativePtr1;
	int xpathArray2[13], ypathArray2[13], ptr2, tentativePtr2;
	int xpathArray3[13], ypathArray3[13], ptr3, tentativePtr3;
	bool gridConflict;
	int xgridArray[12], ygridArray[10];
	bool speedControlEnable, speedControlInit;
	
	//PROCESS: SEND PATH COMMANDS TO ROBOTS
	void sendPathMessage() {
		
		//ROBOT-1 PATH
		for (int i=0; i<13; i++) {
			xpath1.write(xpathArray1[i]);
			ypath1.write(ypathArray1[i]);
			wait(1,SC_US);
		}
		xpath1.write(0);
		ypath1.write(0);
		
		//ROBOT-2 PATH
		for (int i=0; i<13; i++) {
			xpath2.write(xpathArray2[i]);
			ypath2.write(ypathArray2[i]);
			wait(1,SC_US);
		}
		xpath2.write(0);
		ypath2.write(0);
		
		//ROBOT-3 PATH
		for (int i=0; i<13; i++) {
			xpath3.write(xpathArray3[i]);
			ypath3.write(ypathArray3[i]);
			wait(1,SC_US);
		}
		xpath3.write(0);
		ypath3.write(0);
		
	}
	
	//PROCESS: ROBOT-1 SERVER PERMISSIONS
	void checkConflictsRobot1() {
		
		if (xreq1.read()>0 && yreq1.read()>0) {
			
			//Chk for Conflicts
			bool xcondA = xpathArray1[ptr1+1]==xpathArray2[tentativePtr2];
			bool ycondA = ypathArray1[ptr1+1]==ypathArray2[tentativePtr2];
			bool xcondB = xpathArray1[ptr1+1]==xpathArray2[ptr2];
			bool ycondB = ypathArray1[ptr1+1]==ypathArray2[ptr2];
			
			bool xcondC = xpathArray1[ptr1+1]==xpathArray3[tentativePtr3];
			bool ycondC = ypathArray1[ptr1+1]==ypathArray3[tentativePtr3];
			bool xcondD = xpathArray1[ptr1+1]==xpathArray3[ptr3];
			bool ycondD = ypathArray1[ptr1+1]==ypathArray3[ptr3];
			
			gridConflict = (xcondA && ycondA) || (xcondB && ycondB) || (xcondC && ycondC) || (xcondD && ycondD);
			
			//Update
			if (gridConflict) {
				okToCross1.write(2);
			}
			else {
				okToCross1.write(1);
				if (tentativePtr1==ptr1) {
					tentativePtr1++;
				}
			}
		}
		else {
			okToCross1.write(0);
		}
	}
	
	//PROCESS: ROBOT-2 SERVER PERMISSIONS
	void checkConflictsRobot2() {
		
		if (xreq2.read()>0 && yreq2.read()>0) {
			
			//Chk for Conflicts
			bool xcondA = xpathArray2[ptr2+1]==xpathArray1[tentativePtr1];
			bool ycondA = ypathArray2[ptr2+1]==ypathArray1[tentativePtr1];
			bool xcondB = xpathArray2[ptr2+1]==xpathArray1[ptr1];
			bool ycondB = ypathArray2[ptr2+1]==ypathArray1[ptr1];
			
			bool xcondC = xpathArray2[ptr2+1]==xpathArray3[tentativePtr3];
			bool ycondC = ypathArray2[ptr2+1]==ypathArray3[tentativePtr3];
			bool xcondD = xpathArray2[ptr2+1]==xpathArray3[ptr3];
			bool ycondD = ypathArray2[ptr2+1]==ypathArray3[ptr3];
			
			gridConflict = (xcondA && ycondA) || (xcondB && ycondB) || (xcondC && ycondC) || (xcondD && ycondD);
			
			//Update
			if (gridConflict) {
				okToCross2.write(2);
			}
			else {
				okToCross2.write(1);
				if (tentativePtr2==ptr2) {
					tentativePtr2++;
				}
			}
		}
		else {
			okToCross2.write(0);
		}
	}
	
	
	//PROCESS: ROBOT-3 SERVER PERMISSIONS
	void checkConflictsRobot3() {
		
		if (xreq3.read()>0 && yreq3.read()>0) {
			
			//Chk for Conflicts
			bool xcondA = xpathArray3[ptr3+1]==xpathArray1[tentativePtr1];
			bool ycondA = ypathArray3[ptr3+1]==ypathArray1[tentativePtr1];
			bool xcondB = xpathArray3[ptr3+1]==xpathArray1[ptr1];
			bool ycondB = ypathArray3[ptr3+1]==ypathArray1[ptr1];
			
			bool xcondC = xpathArray3[ptr3+1]==xpathArray2[tentativePtr2];
			bool ycondC = ypathArray3[ptr3+1]==ypathArray2[tentativePtr2];
			bool xcondD = xpathArray3[ptr3+1]==xpathArray2[ptr2];
			bool ycondD = ypathArray3[ptr3+1]==ypathArray2[ptr2];
			
			gridConflict = (xcondA && ycondA) || (xcondB && ycondB) || (xcondC && ycondC) || (xcondD && ycondD);
			
			//Update
			if (gridConflict) {
				okToCross3.write(2);
			}
			else {
				okToCross3.write(1);
				if (tentativePtr3==ptr3) {
					tentativePtr3++;
				}
			}
		}
		else {
			okToCross3.write(0);
		}
	}
	

	//PROCESS: UPDATE ROBOT POINTERS
	void updatePointers() {
		
		//ROBOT-1 GRID POINTER
		int nx1 = xpathArray1[ptr1+1];
		int ny1 = ypathArray1[ptr1+1];
		bool xcond1 = x1.read()>xgridArray[nx1-1] && x1.read()<=xgridArray[nx1];
		bool ycond1 = y1.read()>ygridArray[ny1-1] && y1.read()<=ygridArray[ny1];
		if (xcond1 && ycond1) {
			ptr1++;
		}
		
		//ROBOT-2 GRID POINTER
		int nx2 = xpathArray2[ptr2+1];
		int ny2 = ypathArray2[ptr2+1];
		bool xcond2 = x2.read()>xgridArray[nx2-1] && x2.read()<=xgridArray[nx2];
		bool ycond2 = y2.read()>ygridArray[ny2-1] && y2.read()<=ygridArray[ny2];
		if (xcond2 && ycond2) {
			ptr2++;
		}
		
		//ROBOT-3 GRID POINTER
		int nx3 = xpathArray3[ptr3+1];
		int ny3 = ypathArray3[ptr3+1];
		bool xcond3 = x3.read()>xgridArray[nx3-1] && x3.read()<=xgridArray[nx3];
		bool ycond3 = y3.read()>ygridArray[ny3-1] && y3.read()<=ygridArray[ny3];
		if (xcond3 && ycond3) {
			ptr3++;
		}
		
	}
	
	//PROCESS: SPEED CONTROLLER
	void speedCtrl() {
		
		//Robot-1
		if (speedControlInit) {
			speed1.write(0.5);
		}
		else {
			
			//Chk Gridlock Cond
			double d1 = abs(x1.read()-44) + abs(y1.read()-14);
			double d2 = abs(x2.read()-49) + abs(y2.read()-20);
			
			//Robot-1
			if (speedControlEnable) {
				double newSpeed = d1/d2*speed2.read();
				speed1.write(newSpeed);
			}
			
		}
		
		//Robot-2
		speed2.write(1);
		
		//Robot-3
		if (speedControlInit) {
			speed3.write(0.5);
			speedControlInit = false;
		}
		else {
			//Chk Gridlock Cond
			double d2 = abs(x2.read()-32) + abs(y2.read()-24);
			double d3 = abs(x3.read()-44) + abs(y3.read()-24);;
			
			//Robot-3
			if (speedControlEnable) {
				double newSpeed = d3/d2*speed2.read();
				speed3.write(newSpeed);
			}
		}
	}
		
	
	//CONSTRUCTOR
	SC_CTOR(server) {
		
		//PROCESSES
		SC_THREAD(sendPathMessage)
		
		SC_METHOD(checkConflictsRobot1)
		sensitive << clock.pos();
		
		SC_METHOD(checkConflictsRobot2)
		sensitive << clock.pos();
		
		SC_METHOD(checkConflictsRobot3)
		sensitive << clock.pos();
		
		SC_METHOD(updatePointers)
		sensitive << clock.pos();
		
		SC_METHOD(speedCtrl)
		sensitive << clock.pos();

		//Flags, Pointers, etc.
		gridConflict = false;
		ptr1 = 0;
		ptr2 = 0;
		ptr3 = 0;
		tentativePtr1 = 0;
		tentativePtr2 = 0;
		tentativePtr3 = 0;
		
		//ROBOT-1 PATH
		xpathArray1[0] = 2;
		xpathArray1[1] = 3;
		xpathArray1[2] = 3;
		xpathArray1[3] = 3;
		xpathArray1[4] = 4;
		xpathArray1[5] = 5;
		xpathArray1[6] = 6;
		xpathArray1[7] = 7;
		xpathArray1[8] = 8;
		xpathArray1[9] = 9;
		xpathArray1[10] = 10;
		xpathArray1[11] = 11;
		xpathArray1[12] = 11;
		
		ypathArray1[0] = 1;
		ypathArray1[1] = 1;
		ypathArray1[2] = 2;
		ypathArray1[3] = 3;
		ypathArray1[4] = 3;
		ypathArray1[5] = 3;
		ypathArray1[6] = 3;
		ypathArray1[7] = 3;
		ypathArray1[8] = 3;
		ypathArray1[9] = 3;
		ypathArray1[10] = 3;
		ypathArray1[11] = 3;
		ypathArray1[12] = 2;
		
		//ROBOT-2 PATH
		xpathArray2[0] = 1;
		xpathArray2[1] = 2;
		xpathArray2[2] = 3;
		xpathArray2[3] = 4;
		xpathArray2[4] = 5;
		xpathArray2[5] = 6;
		xpathArray2[6] = 7;
		xpathArray2[7] = 8;
		xpathArray2[8] = 9;
		xpathArray2[9] = 9;
		xpathArray2[10] = 9;
		xpathArray2[11] = 10;
		xpathArray2[12] = 11;
		
		ypathArray2[0] = 5;
		ypathArray2[1] = 5;
		ypathArray2[2] = 5;
		ypathArray2[3] = 5;
		ypathArray2[4] = 5;
		ypathArray2[5] = 5;
		ypathArray2[6] = 5;
		ypathArray2[7] = 5;
		ypathArray2[8] = 5;
		ypathArray2[9] = 4;
		ypathArray2[10] = 3;
		ypathArray2[11] = 3;
		ypathArray2[12] = 3;
		
		//ROBOT-3 PATH
		xpathArray3[0] = 11;
		xpathArray3[1] = 10;
		xpathArray3[2] = 9;
		xpathArray3[3] = 8;
		xpathArray3[4] = 7;
		xpathArray3[5] = 7;
		xpathArray3[6] = 7;
		xpathArray3[7] = 8;
		xpathArray3[8] = 9;
		xpathArray3[9] = 9;
		xpathArray3[10] = 9;
		xpathArray3[11] = 10;
		xpathArray3[12] = 11;
		
		ypathArray3[0] = 5;
		ypathArray3[1] = 5;
		ypathArray3[2] = 5;
		ypathArray3[3] = 5;
		ypathArray3[4] = 5;
		ypathArray3[5] = 6;
		ypathArray3[6] = 7;
		ypathArray3[7] = 7;
		ypathArray3[8] = 7;
		ypathArray3[9] = 8;
		ypathArray3[10] = 9;
		ypathArray3[11] = 9;
		ypathArray3[12] = 9;
		
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
		
		//Speed Control Enable
		speedControlEnable = true;
		speedControlInit = true;

		
	}
};

