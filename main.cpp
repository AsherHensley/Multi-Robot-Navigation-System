/*
 *  main.cpp
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

#include "server.h"
#include "sensor.h"
#include "robot.h"
#include "obstacle.h"
#include "printer.h"

int sc_main (int argc, char * argv[]) {
	
	//OBSTACLES
	double xiO1 = 20;
	double yiO1 = 3;
	double vxO1 = -5;
	double vyO1 = 0;
	double dx01 = 16;
	double dy01 = 21;
	
	double xiO2 = 49;
	double yiO2 = 36;
	double vxO2 = -5;
	double vyO2 = 0;
	double dx02 = 45;
	double dy02 = 13;
	
	//TIME PARAMETERS
	double dt = 0.01;	// clock period (seconds)
	double runtime = 90;	// simulation run time (seconds)
	
	//ROBOT-1 SIGNALS
	sc_signal<int> xpath1, ypath1;
	sc_signal<double> x1, y1, xreqIntern1, yreqIntern1, xreq1, yreq1;
	sc_signal<int> okToCross1, obstacleDetected1;
	sc_signal<bool> stoppedFromServer1, stoppedFromSensor1;
	sc_signal<double> speed1;
	
	//ROBOT-2 SIGNALS
	sc_signal<int> xpath2, ypath2;
	sc_signal<double> x2, y2, xreqIntern2, yreqIntern2, xreq2, yreq2;
	sc_signal<int> okToCross2, obstacleDetected2;
	sc_signal<bool> stoppedFromServer2, stoppedFromSensor2;
	sc_signal<double> speed2;
	
	//ROBOT-3 SIGNALS
	sc_signal<int> xpath3, ypath3;
	sc_signal<double> x3, y3, xreqIntern3, yreqIntern3, xreq3, yreq3;
	sc_signal<int> okToCross3, obstacleDetected3;
	sc_signal<bool> stoppedFromServer3, stoppedFromSensor3;
	sc_signal<double> speed3;
	
	//OBSTACLE SIGNALS
	sc_signal<double> xO1, yO1, xO2, yO2;
	
	//CLOCK
	sc_clock clock("CLOCK",dt,SC_SEC,0.5,0,SC_MS,false);
	
	//INSTANTIATIONS
	server server1("SERVER1");
	robot robot1("ROBOT1");
	robot robot2("ROBOT2");
	robot robot3("ROBOT3");
	sensor sensor1("SENSOR1",dt);
	sensor sensor2("SENSOR2",dt);
	sensor sensor3("SENSOR3",dt);
	obstacle obstacle1("OBSTACLE1",xiO1,yiO1,vxO1,vyO1,dt,dx01,dy01);
	obstacle obstacle2("OBSTACLE2",xiO2,yiO2,vxO2,vyO2,dt,dx02,dy02);
	printer printer1("PRINTER1");
	
	//SERVER-1 CONNECTIONS
	server1.clock(clock);
	
	server1.xpath1(xpath1);
	server1.ypath1(ypath1);
	server1.xreq1(xreq1);
	server1.yreq1(yreq1);
	server1.okToCross1(okToCross1);
	
	server1.xpath2(xpath2);
	server1.ypath2(ypath2);
	server1.xreq2(xreq2);
	server1.yreq2(yreq2);
	server1.okToCross2(okToCross2);
	
	server1.xpath3(xpath3);
	server1.ypath3(ypath3);
	server1.xreq3(xreq3);
	server1.yreq3(yreq3);
	server1.okToCross3(okToCross3);
	
	server1.x1(x1);
	server1.y1(y1);
	server1.x2(x2);
	server1.y2(y2);
	server1.x3(x3);
	server1.y3(y3);
	server1.speed1(speed1);
	server1.speed2(speed2);
	server1.speed3(speed3);
	
	//ROBOT-1 CONNECTIONS
	robot1.clock(clock);
	robot1.xreqIn(xreqIntern1);
	robot1.yreqIn(yreqIntern1);
	robot1.xreqOut(xreq1);
	robot1.yreqOut(yreq1);
	robot1.okToCross(okToCross1);
	robot1.stoppedFromServer(stoppedFromServer1);
	robot1.stoppedFromSensor(stoppedFromSensor1);
	robot1.obstacleDetected(obstacleDetected1);
	
	//ROBOT-2 CONNECTIONS
	robot2.clock(clock);
	robot2.xreqIn(xreqIntern2);
	robot2.yreqIn(yreqIntern2);
	robot2.xreqOut(xreq2);
	robot2.yreqOut(yreq2);
	robot2.okToCross(okToCross2);
	robot2.stoppedFromServer(stoppedFromServer2);
	robot2.stoppedFromSensor(stoppedFromSensor2);
	robot2.obstacleDetected(obstacleDetected2);
	
	//ROBOT-3 CONNECTIONS
	robot3.clock(clock);
	robot3.xreqIn(xreqIntern3);
	robot3.yreqIn(yreqIntern3);
	robot3.xreqOut(xreq3);
	robot3.yreqOut(yreq3);
	robot3.okToCross(okToCross3);
	robot3.stoppedFromServer(stoppedFromServer3);
	robot3.stoppedFromSensor(stoppedFromSensor3);
	robot3.obstacleDetected(obstacleDetected3);
	
	//SENSOR-1 CONNECTIONS
	sensor1.clock(clock);
	sensor1.x(x1);
	sensor1.y(y1);
	sensor1.xreq(xreqIntern1);
	sensor1.yreq(yreqIntern1);
	sensor1.xpath(xpath1);
	sensor1.ypath(ypath1);
	sensor1.stoppedFromServer(stoppedFromServer1);
	sensor1.stoppedFromSensor(stoppedFromSensor1);
	sensor1.xO1(xO1);
	sensor1.yO1(yO1);
	sensor1.xO2(xO2);
	sensor1.yO2(yO2);
	sensor1.speed(speed1);
	
	//SENSOR-2 CONNECTIONS
	sensor2.clock(clock);
	sensor2.x(x2);
	sensor2.y(y2);
	sensor2.xreq(xreqIntern2);
	sensor2.yreq(yreqIntern2);
	sensor2.xpath(xpath2);
	sensor2.ypath(ypath2);
	sensor2.stoppedFromServer(stoppedFromServer2);
	sensor2.stoppedFromSensor(stoppedFromSensor2);
	sensor2.xO1(xO1);
	sensor2.yO1(yO1);
	sensor2.xO2(xO2);
	sensor2.yO2(yO2);
	sensor2.speed(speed2);
	
	//SENSOR-3 CONNECTIONS
	sensor3.clock(clock);
	sensor3.x(x3);
	sensor3.y(y3);
	sensor3.xreq(xreqIntern3);
	sensor3.yreq(yreqIntern3);
	sensor3.xpath(xpath3);
	sensor3.ypath(ypath3);
	sensor3.stoppedFromServer(stoppedFromServer3);
	sensor3.stoppedFromSensor(stoppedFromSensor3);
	sensor3.xO1(xO1);
	sensor3.yO1(yO1);
	sensor3.xO2(xO2);
	sensor3.yO2(yO2);
	sensor3.speed(speed3);
	
	//OBSTACLE-1 CONNECTIONS
	obstacle1.clock(clock);
	obstacle1.x(xO1);
	obstacle1.y(yO1);
	
	//OBSTACLE-2 CONNECTIONS
	obstacle2.clock(clock);
	obstacle2.x(xO2);
	obstacle2.y(yO2);
	
	//PRINTER CONNECTIONS
	printer1.clock(clock);
	printer1.x1(x1);
	printer1.y1(y1);
	printer1.x2(x2);
	printer1.y2(y2);
	printer1.x3(x3);
	printer1.y3(y3);
	printer1.xO1(xO1);
	printer1.yO1(yO1);
	printer1.xO2(xO2);
	printer1.yO2(yO2);
	
	//TRACE FILE
	sc_trace_file *tf = sc_create_vcd_trace_file("PHASE1_TEST2");
	sc_trace(tf,clock,"clock");
	sc_trace(tf,x1,"X1");
	sc_trace(tf,y1,"Y1");
	sc_trace(tf,x2,"X2");
	sc_trace(tf,y2,"Y2");
	sc_trace(tf,xO1,"XO1");
	sc_trace(tf,yO1,"YO1");
	sc_trace(tf,xreq1,"XREQ1");
	sc_trace(tf,yreq1,"YREQ1");
	sc_trace(tf,xreq2,"XREQ2");
	sc_trace(tf,yreq2,"YREQ2");
	sc_trace(tf,okToCross1,"OKTOCROSS1");
	sc_trace(tf,okToCross2,"OKTOCROSS2");
	sc_trace(tf,obstacleDetected1,"OBSTACLEDETECTED1");
	sc_trace(tf,obstacleDetected2,"OBSTACLEDETECTED2");
	
	//RUN
    	sc_start(runtime,SC_SEC);
    	return 0;
}
