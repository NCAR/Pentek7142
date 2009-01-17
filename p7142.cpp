#include "p7142.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>


using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142::p7142(std::string devName):
p71xx(devName)
{
}

////////////////////////////////////////////////////////////////////////////////////////
p7142::~p7142() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7142dn::p7142dn(std::string devName, std::string dnName, int bypdiv):
p7142(devName),
_dnName(dnName),
_bypdiv(bypdiv),
_dnFd(-1)
{
	// verify that the card was found
	if (!ok()) {
		std::cerr << "p7142 card not ready" << std::endl;
		return;
	}


	// create the down convertor name
	 _dnName = devName + "/dn/" + _dnName;

	 // open it
	_dnFd = open(_dnName.c_str(), O_RDONLY);
	if (_dnFd < 0) {
		std::cerr << "unable to open " << _dnName << std::endl;
		_ok = false;
		return;
	}

	  // set the clock source
	  int clockSource;

	  //  clockSource = CLK_SRC_FRTPAN;
	  clockSource = CLK_SRC_INTERN;

	  if (ioctl(_dnFd, FIOCLKSRCSET, clockSource) == -1)
	    {
	      std::cerr << "unable to set the clock source for "
			<< _dnName << std::endl;
	      perror("");
	  	_ok = false;
	      return;
	    }

	  // set the clock sample rate
	  double doublearg = 100.0e6;
	  if (ioctl(_dnFd, FIOSAMPRATESET, &doublearg) == -1) {
	    std::cerr << "unable to set the clock rate for "
		      << _dnName << std::endl;
	    perror("");
		_ok = false;
	    return;
	  }

		// set the decimation rate
		if (ioctl(_dnFd, FIOBYPDIVSET, _bypdiv) == -1) {
			std::cerr << "unable to set the bypass decimation rate for "
				  << _dnName << " to " << _bypdiv << std::endl;
			perror("");
			_ok = false;
			return;
		}
			std::cout << "bypass decimation set to " << _bypdiv << std::endl;

	  // flush the device read buffers
	  if (ioctl(_dnFd, FIOFLUSH, 0) == -1)
	    {
	      std::cerr << "unable to flush for "
			<< _dnName << std::endl;
	      perror("");
	  	_ok = false;
	      return;
	    }

		// clear the over/under run counters
		if (overUnderCount() < 0)
		   return;

		_ok = true;

}

////////////////////////////////////////////////////////////////////////////////////////
p7142dn::~p7142dn() {
	if (_dnFd >=0)
		close (_dnFd);
}


///////////////////////////////////////////////////////////
int
p7142dn::overUnderCount() {

	if (!_ok)
		return -1;

  int count = ioctl(_dnFd, FIOGETOVRCNT);
  if (count == -1)
  {
    std::cout << "unable to get ovr/under for "
  	<< _dnName << std::endl;
    perror("");
    _ok = false;
    return -1;
  }

  // clear the overrun counter
  if (ioctl(_dnFd, FIOCLROVRCNT) == -1)
  {
    std::cout << "unable to clear ovr/under for "
  	<< _dnName << std::endl;
    perror("");
    _ok = false;
    return -1;
  }

  return count;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p7142dn::read(char* buf, int bufsize) {

    if (!_ok)
       return -1;

	int n = ::read(_dnFd, buf, bufsize);

	if (n < 0)
		_ok = false;

	return n;

}

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
