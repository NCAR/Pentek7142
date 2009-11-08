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