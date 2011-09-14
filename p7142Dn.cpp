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
        p7142* p7142,
        int chanId,
        int bypassdivrate,
        int simWaveLength,
        bool sim4bytes,
        bool internalClock):
  _p7142(*p7142),
  _chanId(chanId),
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

    _dnName = _p7142.devName();

    p7142->start(chanId);

    // Set the clock source.
    int clockSource = internalClock ? CLK_SRC_INTERN : CLK_SRC_FRTPAN;

    /// @todo Figure out how to set the clock source to internal under ReadyFlow
    /**
    if (ioctl(_dnFd, FIOCLKSRCSET, clockSource) == -1) {
        std::cerr << "unable to set the clock source for "
                << _dnName << std::endl;
        abort();
    }
    **/
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn::~p7142Dn() {
  boost::recursive_mutex::scoped_lock guard(_mutex);
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

	int count = 0;
    // if simulate, indicate no errors.
    if (isSimulating()) {
        return count;
    }

    /// @todo Need to track down how the overflow count was
    /// done in the pentek driver, and migrate to ReadyFlow usage here.
    /**
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
     **/

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
        // not in simulation mode; do a proper read from the device
        int n = _p7142.read(_chanId, buf, bufsize);

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
bool
p7142Dn::usingInternalClock() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return(false);
    
    uint32_t clkSel;

    P7142_GET_MSTR_BUS_CTRL_SEL_CLK(_p7142.p7142Regs.BAR2RegAddr.masterAControl, clkSel);
    return (clkSel == P7142_MSTR_CTRL_SEL_CLK_EXT_CLK);

}

////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::bypassDivider() const {

	if (isSimulating())
        return(0);
    
    return (*_p7142.p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide) + 1;
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
}
