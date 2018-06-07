/*
 *  printer.h
 *
 *  Printer module: contains processes and constructor for data print out
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
#include <iomanip>
using namespace std;


SC_MODULE(printer) {
	
	sc_in<bool> clock;
	sc_in<double> x1, y1, x2, y2, x3, y3, xO1, yO1 ,xO2, yO2;
	
	int spc;
	
	void updatePrintout() {
		
		//HEADER
		cout << x1.read() << setw(spc);
		cout << y1.read() << setw(spc);
		cout << x2.read() << setw(spc);
		cout << y2.read() << setw(spc);
		cout << x3.read() << setw(spc);
		cout << y3.read() << setw(spc);
		cout << xO1.read() << setw(spc);
		cout << yO1.read() << setw(spc);
		cout << xO2.read() << setw(spc);
		cout << yO2.read() << endl;
		
	}
	
	SC_CTOR(printer) {
		
		SC_METHOD(updatePrintout);
		sensitive << clock.pos();
		
		spc = 12;
		
		//HEADER
		cout << "X1" << setw(spc);
		cout << "Y1" << setw(spc);
		cout << "X2" << setw(spc);
		cout << "Y2" << setw(spc);
		cout << "X3" << setw(spc);
		cout << "Y3" << setw(spc);
		cout << "XO1" << setw(spc);
		cout << "YO1" << setw(spc);
		cout << "XO2" << setw(spc);
		cout << "YO2" << endl;

	}

};