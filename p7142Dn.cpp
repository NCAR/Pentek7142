/*
 * p7142Dn.cpp
 *
 *  Created on: Oct 5, 2010
 *      Author: burghart
 */
#include "p7142Dn.h"
#include "p7142.h"

#include <fcntl.h>
#include <iostream>
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/ioctl.h>

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////
p7142Dn::p7142Dn(
        p7142 * p7142,
        int chanId,
        int bypassdivrate,
        int simWaveLength,
        bool sim4bytes,
        bool internalClock):
  _p7142(*p7142),
  _chanId(chanId),
  _dnFd(-1),
  _simWaveLength(simWaveLength),
  _angleCount(0),
  _sim4bytes(sim4bytes),
  _mutex()
{
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
        _dnName = "dnSimulate";
        return;
    }

    p7142->start(chanId);

    return;
    // create the downconverter device name
    char c[2];
    c[0] = '0' + _chanId;
    c[1] = 0;
    std::string dnchan(c);
    dnchan += "BR";

    _dnName = _p7142.devName() + "/dn/" + dnchan;

    // open it
    _dnFd = open(_dnName.c_str(), O_RDWR);
    if (_dnFd < 0) {
        std::cerr << "unable to open " << _dnName << std::endl;
        perror("error opening pentek device:");
        abort();
    }

    // Set the clock source.
    int clockSource = internalClock ? CLK_SRC_INTERN : CLK_SRC_FRTPAN;

    if (ioctl(_dnFd, FIOCLKSRCSET, clockSource) == -1) {
        std::cerr << "unable to set the clock source for "
                << _dnName << std::endl;
        abort();
    }
    // Using our firmware, we *must* reset the Pentek DCM after changing 
    // clock source
    _p7142.resetDCM();

    // set the bypass divider
    if (! setBypassDivider(bypassdivrate)) {
        abort();
    }

    // flush the device read buffers
    if (ioctl(_dnFd, FIOFLUSH, 0) == -1) {
        std::cerr << "unable to flush for " << _dnName << std::endl;
        abort();
    }

    // clear the over/under run counters
    overUnderCount();
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn::~p7142Dn() {
  boost::recursive_mutex::scoped_lock guard(_mutex);
  
  if (_dnFd >= 0)
    close (_dnFd);
}

////////////////////////////////////////////////////////////////////////////////
std::string
p7142Dn::dnName() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    return _dnName;
}

////////////////////////////////////////////////////////////////////////////////
bool
p7142Dn::isSimulating() const {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    return _p7142.isSimulating();
}

////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::overUnderCount() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);

    // if simulate, indicate no errors.
    if (isSimulating()) {
        return 0;
    }

    return 0;

    int count = ioctl(_dnFd, FIOGETOVRCNT);
    if (count == -1) {
        std::cout << "unable to get ovr/under for "
                << _dnName << std::endl;
        perror("");
        return -1;
    }

    // clear the overrun counter
    if (ioctl(_dnFd, FIOCLROVRCNT) == -1) {
        std::cout << "unable to clear ovr/under for "
                << _dnName << std::endl;
        perror("");
        return -1;
    }

    return count;
}

////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::read(char* buf, int bufsize) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Enforce that reads are a multiple of 4 bytes, since the Pentek driver
    // (silently) does this, e.g., it will return 4 bytes if 7 are requested,
    // or 0 bytes if 1 is requested.
    // If we are to support other read sizes, we'll have to be a lot smarter, 
    // and keep around a buffer of up to 3 unconsumed bytes between calls here. 
    // For now, we are not that smart...
    if ((bufsize % 4) != 0) {
        std::cerr << __PRETTY_FUNCTION__ << ": " << bufsize << 
                " bytes requested, but Pentek reads must be a multiple of 4 bytes!" <<
                std::endl;
        abort();
    }

    if (!isSimulating()) {
    	return bufsize;

        // not in simulation mode; do a proper read from the device
        int n = ::read(_dnFd, buf, bufsize);

        if (n > 0)
            _bytesRead += n;

        return n;
    }

    // In simulation mode, create some random values
    // and return a full buffer.

    // there is a bug in this code. It assumes that bufsize
    // is an integral number of I/Q pairs. For the time being,
    // detect if this is not true and just abort.
    /// @todo Fix read function to return an arbitrary number of bytes
    /// when in simulation mode
    assert ((bufsize % 4) == 0);

    short* sbuf = (int16_t*)buf;
    int*   ibuf = (int32_t*)buf;

    // 4  or 8 bytes per IQ pair
    int nPairs;
    if (_sim4bytes) {
        nPairs = (bufsize) / 8;
    } else {
        nPairs = (bufsize) / 4;
    }
    for (int p = 0; p < nPairs; p++) {
        // noise is +/-10% amplitude
        double noise = 0.1 * ((2.0 * rand()) / RAND_MAX - 1.0);
        // Noisy sine wave, with wavelength of _simWaveLength gates
        // The wavelength varies across the range
        if (_angleCount == _simWaveLength) {
            _angleCount = 0;
        }

        double angle = ((double)_angleCount++)/ _simWaveLength;
        double I = 10000 * (sin((2 * angle * M_PI)) + noise);
        double Q = 10000 * (cos((2 * angle * M_PI)) + noise);
        if (_sim4bytes) {
            *ibuf++ = (int32_t) I; // I
            *ibuf++ = (int32_t) Q; // Q
        } else {
            *sbuf++ = (int16_t) I; // I
            *sbuf++ = (int16_t) Q; // Q
        }
    }
    _bytesRead += bufsize;

    return bufsize;
}

////////////////////////////////////////////////////////////////////////////////
long
p7142Dn::bytesRead() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    long retval = _bytesRead;
    _bytesRead = 0;
    return retval;
}

////////////////////////////////////////////////////////////////////////////////
void p7142Dn::flush() {
  boost::recursive_mutex::scoped_lock guard(_mutex);
  
  // flush the device read buffers. This will clear the fifos,
  // which will probably contain data since we are not able yet
  // to disable the timers, and so the fifos may have been
  // filling. When we do implement true timer control, this
  // flush will probably not be needed.
  if (isSimulating())
      return;
  
  /// @todo Reset the A/D FIFOS. Do we need to do some sort of re-initialization
  /// of the DMA?

  //if (ioctl(_dnFd, FIOFLUSH, 0) == -1) {
  //  std::cerr << "unable to flush for " << _dnName << std::endl;
  //  perror("");
  //}

  std::cout << "flush performed on " << _dnName << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
bool
p7142Dn::usingInternalClock() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return(false);
    
    uint32_t clkSel;

    P7142_GET_MSTR_BUS_CTRL_SEL_CLK(_p7142.p7142Regs.BAR2RegAddr.masterAControl, clkSel);
    return (clkSel == P7142_MSTR_CTRL_SEL_CLK_EXT_CLK);

    //int clockSource;
    //if ((clockSource = ioctl(_dnFd, FIOCLKSRCGET, 0)) == -1) {
    //    std::cerr << __FUNCTION__ << ": ioctl error on FIOCLKSRCGET: " <<
    //            strerror(errno);
    //}
    //return(clockSource == CLK_SRC_INTERN);
}

////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::bypassDivider() const {
    //boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return(0);
    
    return (*_p7142.p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide) + 1;

    //int bypassdiv;
    //if ((bypassdiv = ioctl(_dnFd, FIOBYPDIVGET, 0)) == -1) {
    //  std::cerr << "Unable to get the bypass divider for "
    //        << _dnName << std::endl;
    //  perror(__FUNCTION__);
    //}
}

////////////////////////////////////////////////////////////////////////////////
bool
p7142Dn::setBypassDivider(int bypassdiv) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return true;
    
    *_p7142.p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide = (bypassdiv-1);

    if (*_p7142.p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide != (bypassdiv-1)) {
    	return false;
    }

    return true;


    // set the bypass divider
    if (ioctl(_dnFd, FIOBYPDIVSET, bypassdiv) == -1) {
      std::cerr << "unable to set the bypass divider for "
            << _dnName << " to " << bypassdiv << std::endl;
      perror(__FUNCTION__);
      return false;
    }
    return true;
}
