// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=* 
// ** Copyright UCAR (c) 1990 - 2016                                         
// ** University Corporation for Atmospheric Research (UCAR)                 
// ** National Center for Atmospheric Research (NCAR)                        
// ** Boulder, Colorado, USA                                                 
// ** BSD licence applies - redistribution and use in source and binary      
// ** forms, with or without modification, are permitted provided that       
// ** the following conditions are met:                                      
// ** 1) If the software is modified to produce derivative works,            
// ** such modified software should be clearly marked, so as not             
// ** to confuse it with the version available from UCAR.                    
// ** 2) Redistributions of source code must retain the above copyright      
// ** notice, this list of conditions and the following disclaimer.          
// ** 3) Redistributions in binary form must reproduce the above copyright   
// ** notice, this list of conditions and the following disclaimer in the    
// ** documentation and/or other materials provided with the distribution.   
// ** 4) Neither the name of UCAR nor the names of its contributors,         
// ** if any, may be used to endorse or promote products derived from        
// ** this software without specific prior written permission.               
// ** DISCLAIMER: THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS  
// ** OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED      
// ** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.    
// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=* 
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <sched.h>
#include <sys/timeb.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "p7142.h"
#include "p7142Up.h"

#define BASICSIZE   1024

using namespace std;

bool stop = false;
///////////////////////////////////////////////////////////
void handler( int signal ) {
	stop = true;
}

///////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	if (argc < 6) {
		std::cout << "usage: " << argv[0] << "\n" << "\
    <device root (e.g. /dev/pentek/0/)>\n\
    <up converter name (e.g. 0C)>\n\
    <sample rate Hz>\n\
    <nco frequency Hz>\n\
    <cm_mode 0-15 (The DAC CONFIG2 coarse mixer mode, see DAC5687 data sheet.)>\n";
		exit(1);
	}

	std::string devRoot = argv[1];
	std::string upName  = argv[2];
	double sampleRate   = atof(argv[3]);
	double ncoFreq      = atof(argv[4]);
	char mode           = atoi(argv[5]);

	// create the p7142 and its upconverter
	Pentek::p7142 card(65536, false);
	Pentek::p7142Up & upConverter = *card.addUpconverter(sampleRate, ncoFreq, mode);

	// create the signal
	unsigned int n = 100;
	int32_t IQ[n];
	for (unsigned int i = 0; i < n/4; i++) {
		IQ[i]   = 0x7FFF << 16 | 0x7FFF;
	}
	for (unsigned int i = n/4; i < n/2; i++) {
		IQ[i]   = 0x8000 << 16 | 0x8000;
	}
    for (unsigned int i = n/2; i < n; i++) {
		IQ[i]   = 0;
	}


   for (unsigned int i = 0; i < n; i++) {
		IQ[i]   = 0x7FFF << 16 | 0x7FFF;
	}

	// load mem2
	upConverter.write(IQ, n);

	// start the upconverter

	upConverter.startDAC();

	signal( SIGINT, handler );
	while (1) {
		std::cout << ".";
		std::cout << std::flush;
		sleep(1);
		if (stop)
			break;
	}

	upConverter.stopDAC();
	std::cout << std::endl;
	std::cout << "DAC stopped" << std::endl;

}

