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
/*
 * SingleMutex.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: martinc
 */
#include "SingleMutex.h"

////////////////////////////////////////////////////////////////////////////////////////
SingleMutex* SingleMutex::_instance = 0;
pthread_mutex_t* SingleMutex::_m = new pthread_mutex_t;
////////////////////////////////////////////////////////////////////////////////////////
SingleMutex::SingleMutex() {
}
////////////////////////////////////////////////////////////////////////////////////////
SingleMutex*
SingleMutex::create() {
	if (_instance == NULL) {
		_instance = new SingleMutex();
		pthread_mutex_init(_m, NULL);
	}
	return _instance;
}
////////////////////////////////////////////////////////////////////////////////////////
SingleMutex::~SingleMutex() {
	if (!_instance) {
		return;
	}
	pthread_mutex_destroy(_m);
	delete _m;
	delete _instance;
	_instance = NULL;
}
////////////////////////////////////////////////////////////////////////////////////////
void
SingleMutex::lock() {
	pthread_mutex_lock(_m);
}
////////////////////////////////////////////////////////////////////////////////////////
void
SingleMutex::unlock() {
	pthread_mutex_unlock(_m);
}
