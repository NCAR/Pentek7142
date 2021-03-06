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
#include "p7142sd3c.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdlib>

#include <logx/Logging.h>
LOGGING("p7142sd3c");

namespace Pentek {
    
using namespace boost::posix_time;

/*
 * Timer identifier bits for the SD3C timers.
 */
const unsigned int p7142sd3c::SD3C_TIMER_BITS[N_SD3C_TIMERS] = {
        0x010, 0x020, 0x040, 0x080,
        0x100, 0x200, 0x400, 0x800
};
const unsigned int p7142sd3c::ALL_SD3C_TIMER_BITS = 0xff0;

// 1970-01-01 00:00:00 UTC

const ptime p7142sd3c::Epoch1970(boost::gregorian::date(1970, 1, 1), 
                                 time_duration(0, 0, 0));

///////////////////////////////////////////////////////////////////////////////////////
p7142sd3c::p7142sd3c(bool simulate, double tx_delay,
    double tx_pulsewidth, double prt, double prt2, bool staggeredPrt, 
    unsigned int gates, unsigned int nsum, bool freeRun, 
    DDCDECIMATETYPE requiredDDCType, bool externalStartTrigger, double simPauseMS,
    bool useFirstCard,
    bool rim,
    int codeLength,
    double adc_clock,
    bool reset_clock_managers /* = true */,
    bool abortOnError /* = true */,
    uint16_t sd3cTimerDivisor /* = 2 */) :
        p7142(simulate, simPauseMS, useFirstCard),
        _abortOnError(abortOnError),
        _constructorOk(true),
        _staggeredPrt(staggeredPrt),
        _freeRun(freeRun),
        _prt(prt),
        _prt2(prt2),
        _gates(gates),
        _nsum(nsum),
        _ddcType(DDCUNDEFINED),
        _externalStartTrigger(externalStartTrigger),
        _rim(rim),
        _codeLength(codeLength),
        _motorZeroPositionSet(false),
        _sd3cTimerDivisor(sd3cTimerDivisor) {

	boost::recursive_mutex::scoped_lock guard(_p7142Mutex);
	// If the p7142 constructor had a problem, just return now
	if (! _isReady) {
	    return;
	}
    // sanity check
    if (_nsum < 1) {
        std::cerr << "Selected nsum of " << _nsum << " makes no sense!" <<
                std::endl;
        if (_abortOnError) {
          abort();
        }
        _constructorOk = false;
        return;
    }

    // Get the firmware revision and DDC type from the FPGA.
	if (simulate) {
		if (!_rim) {
			_sd3cRev = 1;
		} else {
			_sd3cRev = 10007;
		}
		_ddcType = requiredDDCType;
        _firmwareVersion = 0;
	} else {
		_sd3cRev = _unpackSd3cRev();
		_ddcType = _unpackDdcType();
		if (_ddcType != requiredDDCType) {
		    ELOG << "Card's DDC type (" << ddcTypeName(_ddcType) <<
		            ") != required DDC type (" <<
		            ddcTypeName(requiredDDCType) << ")";
	        if (abortOnError) {
	            abort();
	        } else {
	            _constructorOk = false;
	        }
		}
	}
    
    if (adc_clock == 0) {
      _adcClock = DefaultAdcFrequency(_ddcType);
    } else {
      // set from the parameter passed in
      _adcClock = adc_clock;
    }

    // Validate _sd3cTimerDivisor. We only accept values of 2, 4, 8, or 16.
    if (_sd3cTimerDivisor != 2 && _sd3cTimerDivisor != 4 &&
            _sd3cTimerDivisor != 8 && _sd3cTimerDivisor != 16) {
        ELOG << "Bad SD3C timer divisor: " << _sd3cTimerDivisor <<
                ". Only 2, 4, 8, and 16 are allowed.";
        if (abortOnError) {
            abort();
        } else {
            _constructorOk = false;
        }
    }

    // Announce the FPGA firmware revision
    ILOG << "7142 card " << _cardIndex << " has SD3C " <<
    		ddcTypeName() << " rev. " << std::dec << _sd3cRev;
    if (_sd3cRev == 0) {
        WLOG << "SD3C revision number is 0. Was the correct firmware loaded?";
    }

    // Determine our operating mode. Use a heuristic here,
    // since the factors determining the mode are not mutually
    // exclusive.
    /// @todo refactor so that the mode is simply set in the constructor.
    _mode = (_nsum > 1) ? MODE_CI : MODE_PULSETAG;

    if (_rim) {
    	_mode = MODE_CI_RIM;
    }

    if (_freeRun) {
        _mode = MODE_FREERUN;
    }

    // stop the timers
    timersStartStop(false);

    // Read the PCI FPGA code revision register, and verify that it is
    // from 2007-11-30 (or later).
    if (! simulate) {
        uint32_t pciFpgaRev;
        P7142_REG_READ(_p7142Regs.BAR0RegAddr.fpgaRevision, pciFpgaRev);
        // The upper 24 bits of pciFpgaRev hold the revision date in weird
        // binary coded decimal form, i.e., 2007-11-30 becomes the 24-bit 
        // value 0x071130. We add 0x20000000 to get the full 4-digit year,
        // i.e., 0x20071130.
        uint32_t revDate = (pciFpgaRev >> 8) + 0x20000000;
        DLOG << "Card " << _cardIndex << " PCI FPGA rev date " << std::hex << 
            revDate << std::dec;
        if (revDate < 0x20071130) {
            ELOG << "Card " << _cardIndex << 
                " has Pentek PCI FPGA code dated " <<
                std::hex << revDate << std::dec <<
                ", but 20071130 or later is required.";
            if (abortOnError) {
              abort();
            }
            _constructorOk = false;
            return;
        }
    }
    
    // Write the gate count and coherent integration registers
    if (! isSimulating()) {
    	uint32_t temp;

    	P7142_REG_WRITE(_BAR2Base + RADAR_GATES, gates);
        usleep(p7142::P7142_IOCTLSLEEPUS);
    	P7142_REG_READ (_BAR2Base + RADAR_GATES, temp);
    	DLOG << "RADAR_GATES readback is " << temp;

    	// nsum tells the firmware if we are in coherent integrator mode,
    	// and if so, the number of sums performed by each of the paired
    	// integrators. If nsum > 1, then we are in coherent integrator mode.
    	// In this case, divide it in half, because thats the number that each
    	// (even, odd pulse) integrator will need.
    	/// @todo Fix the VHDL code, so that it performs the division
    	/// by two, rather than doing it here.
    	P7142_REG_WRITE(_BAR2Base + CI_NSUM, (_nsum < 2 ? _nsum : _nsum/2));
        usleep(p7142::P7142_IOCTLSLEEPUS);
    	P7142_REG_READ (_BAR2Base + CI_NSUM, temp);
    	DLOG << "CI_NSUM readback is " << temp;

        // Bit zero of the TTL_OUT1 register enables zeroing of the counts for
        // all motors being monitored. At startup, we want this bit cleared.
        DLOG << "Clearing the 'zero motor counts' bit";
        uint16_t regVal = TTLIn();
        TTLOut(regVal & ~0xfffe);
        usleep(p7142::P7142_IOCTLSLEEPUS);
    }

    // Convert prt, prt2, tx_pulsewidth, and tx_delay into SD3C timer counts
    _prtCounts = timeToCounts(_prt);
    _prt2Counts = timeToCounts(_prt2);
    _prf = 1.0 / _prt;   // Hz
    _prf2 = 1.0 / _prt2; // Hz

    // Sync pulse timer. Note that the width of this timer must be at least
    // 140 ns to be counted by the Acromag PMC730 multi-IO card pulse counter,
    // and this counter is used by the Ka-band radar! We use 200 ns to be safe.
    setTimer(MASTER_SYNC_TIMER, 0, timeToCounts(200.e-9), true);
    
    // tx pulse timer
    int txDelayCounts = timeToCounts(tx_delay);
    int pulseWidthCounts = timeToCounts(tx_pulsewidth);
    setTimer(TX_PULSE_TIMER, txDelayCounts, pulseWidthCounts, true);

    // log startup params in debug mode

    DLOG << "=============================";

    DLOG << "p7142sd3c constructor";
    DLOG << "  abortOnError: " << (_abortOnError? "Y":"N");
    DLOG << "  downconverter: " << ddcTypeName(_ddcType);
    DLOG << "  simulate: " << simulate;
    DLOG << "  tx_delay: " << tx_delay;
    DLOG << "  tx_pulsewidth: " << tx_pulsewidth;
    DLOG << "  prt: " << 1.0e6 * _prt << " us";
    DLOG << "  prt2: " << 1.0e6 * _prt2 << " us";
    DLOG << "  prf: " << _prf << " Hz";
    DLOG << "  prf2: " << _prf2 << " Hz";
    DLOG << "  prtCounts: " << _prtCounts << " SD3C clock counts";
    DLOG << "  prt2Counts: " << _prt2Counts << " SD3C clock counts";
    DLOG << "  staggeredPrt: " << staggeredPrt;
    DLOG << "  gates: " << gates;
    DLOG << "  nsum: " << nsum;
    DLOG << "  freeRun: " << ((freeRun) ? "true" : "false");
    DLOG << "  requiredDDCType: " << requiredDDCType;
    DLOG << "  externalStartTrigger: " << externalStartTrigger;
    DLOG << "  simPauseMS: " << simPauseMS;
    DLOG << "  useFirstCard: " << useFirstCard;
    DLOG << "  cardIndex: " << _cardIndex;

    DLOG << "  tx delay: "
         << timerDelay(TX_PULSE_TIMER) << " SD3C clock counts" ;
    DLOG << "  tx pulse width: " 
         << timerWidth(TX_PULSE_TIMER) << " SD3C clock counts";
    DLOG << "  staggered: " << ((_staggeredPrt) ? "true" : "false");
    DLOG << "  adc clock: " << 1.0e-6 * adcFrequency() << " MHz";
    DLOG << "  SD3C clock: " << 1.0e-6 * _sd3cFrequency() << " MHz";
    DLOG << "  data rate: " << 1.0e-3 * dataRate() << " KB/s";
    DLOG << "  timerDivisor: " << _sd3cTimerDivisor;

    DLOG << "=============================";

    if (reset_clock_managers) {
      resetClockManagers();
    }

}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3c::~p7142sd3c() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn*
p7142sd3c::addDownconverter(int chanId, uint32_t dmaDescSize, 
        bool burstSampling, int tsLength, double rx_delay, 
        double rx_pulse_width, std::string gaussianFile, 
        std::string kaiserFile, int simWavelength,
        bool internalClock) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    uint32_t maxDmaSize = 1024 * 1024 * 4;
    if (dmaDescSize > maxDmaSize) {
      ELOG << "ERROR in addDownconverter";
      ELOG << "  Requested DMA buffer size too large: " << dmaDescSize;
      ELOG << "  Max allowable size 4 MBytes: " << maxDmaSize;
      if (_abortOnError) {
        abort();
      }
      return NULL;
    }

    // Create a new p7142sd3cDn downconverter and put it in our list
    p7142sd3cDn* downconverter =
      new p7142sd3cDn(this,
                      chanId,
                      dmaDescSize,
                      burstSampling,
                      tsLength,
                      rx_delay,
                      rx_pulse_width,
                      gaussianFile,
                      kaiserFile,
                      simWavelength,
                      internalClock,
                      _abortOnError);

    if (!downconverter->constructorOk()) {
      return NULL;
    }

    p7142::_addDownconverter(downconverter);

    return(downconverter);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142sd3c::setTimer(TimerIndex ndx, int delay, int width, bool verbose, bool invert) {

    _TimerConfig currentVals = _timerConfigs[ndx];

    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    // If current timer width is non-zero, warn about any changes in 
    // width or delay.
    if (verbose && currentVals.width() != 0) {
        if (currentVals.width() != width) {
            std::cerr << "WARNING: Width for timer " << ndx << 
                    " is changing from " << currentVals.width() << " to " <<
                    width << std::endl;
        }
        if (currentVals.delay() != delay) {
            std::cerr << "WARNING: Delay for timer " << ndx << 
                    " is changing from " << currentVals.delay() << " to " <<
                    delay << std::endl;
        }
    }

    _timerConfigs[ndx] = _TimerConfig(delay, width, invert, false);

    DLOG << "--------------------------";
    DLOG << "Setting up timer, ndx: " << ndx;
    DLOG << "  delay: " << delay;
    DLOG << "  width: " << width;
    DLOG << "  invert: " << invert;
    DLOG << "--------------------------";
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3c::timeToCounts(double time) const {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);
    return(lround(time * _sd3cFrequency()));
}

//////////////////////////////////////////////////////////////////////////////////
double
p7142sd3c::countsToTime(int counts) const {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);
    return(counts / _sd3cFrequency());
}

/////////////////////////////////////////////////////////////////////////
bool p7142sd3c::timersStartStop(bool start) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate) {
        setXmitStartTime(microsec_clock::universal_time());
        return true;
    }

    // Set enabled/disabled state for all timers
    for (int i = 0; i < N_SD3C_TIMERS; i++) {
        _timerConfigs[i].setEnabled(start);
    }

    // (Re)initialize timers to establish enabled/disabled state
    if (! initTimers(start)) {
      ELOG << "***** ERROR timersStartStop(), cannot init timers *****";
      return false;
    }
        
    // Turn on Write Strobes
    P7142_REG_WRITE(_BAR2Base + MT_WR, WRITE_ON);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    // Get current time
    ptime now(microsec_clock::universal_time());
    //
    // Actually start or stop the timers now
    //
    if (start) {
        DLOG << "About to start timers: " << now;
        if (_externalStartTrigger) {
            // We assume here that the external trigger is a 1 PPS signal, 
            // e.g., from GPS.
            //
            // Sleep until ~0.5 seconds after the top of a second. This gives
            // us a comfortable fraction of a second to set up timer start and 
            // know at precisely which second the timers will start. It also 
            // allows for our system clock to be off by up to 0.2 seconds.
            
            // sleep until the next 0.5 second mark
            int wake_uSec = 500000; // wake at 0.5 seconds after the top of a second
            int usecNow = now.time_of_day().total_microseconds() % 1000000;
            int sleep_uSec = (1000000 + wake_uSec - usecNow) % 1000000;
            // Timers will start at the top of the next second after we wake
            setXmitStartTime(now + microseconds(1000000 + sleep_uSec - wake_uSec));
            // Now sleep
            usleep(sleep_uSec);
            ptime beforeStart(microsec_clock::universal_time());
            DLOG << "Time just before start: " << beforeStart;
            // Set the wait-for-trigger bit so timers start at the next
            // trigger.
            P7142_REG_WRITE(_BAR2Base + MT_ADDR, ALL_SD3C_TIMER_BITS | GPS_EN);
            usleep(p7142::P7142_IOCTLSLEEPUS);
            ptime afterStart(microsec_clock::universal_time());
        } else {
            // Internal trigger: timers start immediately.
            setXmitStartTime(now);
            P7142_REG_WRITE(_BAR2Base + MT_ADDR, ALL_SD3C_TIMER_BITS | ADDR_TRIG);
            usleep(p7142::P7142_IOCTLSLEEPUS);
        }
        DLOG << "Timers/radar start time " << _radarStartTime;
    } else {
    	P7142_REG_WRITE(_BAR2Base + MT_ADDR, ALL_SD3C_TIMER_BITS);
        usleep(p7142::P7142_IOCTLSLEEPUS);
        DLOG << "Timers stopped at " << now;
    }
    
    // Turn off Write Strobes
    P7142_REG_WRITE(_BAR2Base + MT_WR, WRITE_OFF);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    return true;

}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::startFilters() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (isSimulating())
        return;

    // Enable all adc fifos by setting the global gate enable
    enableGateGen();

    // Start the DDC coefficient counters
    P7142_REG_WRITE(_BAR2Base + KAISER_ADDR, DDC_START);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    DLOG << "fifos and filters enabled";
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::stopFilters() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (isSimulating())
        return;

    uint32_t temp;
    // stop the filters if they are running.
    P7142_REG_READ (_BAR2Base + KAISER_ADDR, temp);
    usleep(p7142::P7142_IOCTLSLEEPUS);
    P7142_REG_WRITE(_BAR2Base + KAISER_ADDR, DDC_STOP);
    usleep(p7142::P7142_IOCTLSLEEPUS);
    P7142_REG_READ (_BAR2Base + KAISER_ADDR, temp);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    // Disable all adc fifos by clearing the global gate enable
    disableGateGen();

    DLOG << "fifos and filters disabled, temp: " << temp;
}

//////////////////////////////////////////////////////////////////////
/// Reset the clock managers
/// set free run mode
/// and stop filters

void p7142sd3c::resetClockManagers()

{

  // reset the FPGA clock managers. Necessary since some of our
  // new DCMs in the firmware use the CLKFX output, which won't
  // lock at startup.
  resetDCM();
  
  // set free run mode as appropriate
  loadFreeRun();
  
  // stop the filters; to be started at the appropriate time by the user.
  stopFilters();
  
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3c::rim() {
	return _rim;
}

//////////////////////////////////////////////////////////////////////
unsigned short int p7142sd3c::TTLIn() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate)
        return 0;

    uint32_t val;
    P7142_REG_READ(_BAR2Base + TTL_IN, val);

    return val;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::TTLOut(unsigned short int data) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate)
        return;

    P7142_REG_WRITE(_BAR2Base + TTL_OUT1, data);

}

//////////////////////////////////////////////////////////////////////
unsigned int p7142sd3c::_sd3cTypeAndRev() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate)
        return 1;

    uint32_t retval;

    P7142_REG_READ(_BAR2Base + FPGA_REPO_REV, retval);

    return retval;

}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::_unpackSd3cRev() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate)
        return _ddcType;

    unsigned int ddcTypeAndRev = _sd3cTypeAndRev();

    // Up to rev 502, DDC type was a 1-bit value at bit 15.
    // After that it's a 2-bit value in bits 14-15.
    int retval = ((ddcTypeAndRev & 0x3fff) > 502) ?
        (ddcTypeAndRev & 0x3fff) : (ddcTypeAndRev & 0x7fff);

    return retval;
}

//////////////////////////////////////////////////////////////////////
p7142sd3c::DDCDECIMATETYPE p7142sd3c::_unpackDdcType() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate)
        return _ddcType;
    
    unsigned int ddcTypeAndRev = _sd3cTypeAndRev();

    // Up to rev 502, DDC type was a 1-bit value at bit 15.
    // After 502 it's a 2-bit value in bits 14-15.
    // After 1120 it's a 3-bit value in bits 13-15.

    int ddcTypeCode = 0;
    int nTypeBits = 0;

    if ((ddcTypeAndRev & 0x1fff) >= 1120) {
      // later version, type is 3 bits
      nTypeBits = 3;
      ddcTypeCode = (ddcTypeAndRev & 0xE000) >> 13;
      _firmwareVersion = ddcTypeAndRev & 0x1fff;
    } else if ((ddcTypeAndRev & 0x3fff) > 502) {
      // intermediate version, type is 2 bits
      nTypeBits = 2;
      ddcTypeCode = (ddcTypeAndRev & 0xC000) >> 14;
      _firmwareVersion = ddcTypeAndRev & 0x3fff;
    } else {
      // early version, type is 1 bit
      nTypeBits = 1;
      ddcTypeCode = (ddcTypeAndRev & 0x8000) >> 15;
      _firmwareVersion = ddcTypeAndRev & 0x7fff;
    }

    static DDCDECIMATETYPE oneBitMapping[] = {
            DDC4DECIMATE,   // 0
            DDC8DECIMATE    // 1
    };
    static DDCDECIMATETYPE twoBitMapping[] = {
            DDC4DECIMATE,   // 0
            DDC8DECIMATE,   // 1
            DDC10DECIMATE,  // 2
            BURST,          // 3
    };
    static DDCDECIMATETYPE threeBitMapping[] = {
            DDC4DECIMATE,   // 0
            DDCUNDEFINED,   // 1 (currently unused)
            DDC8DECIMATE,   // 2
            DDCUNDEFINED,   // 3 (currently unused)
            DDC10DECIMATE,  // 4
            DDCUNDEFINED,   // 5 (currently unused)
            BURST,          // 6
            DDC6DECIMATE    // 7
    };

    DLOG << "Firmware version: " << _firmwareVersion;
    DLOG << "DDC Type code (" << nTypeBits << "-bit): " << ddcTypeCode;

    DDCDECIMATETYPE retval;
    switch (nTypeBits) {
    case 1:
        retval = oneBitMapping[ddcTypeCode];
        break;
    case 2:
        retval = twoBitMapping[ddcTypeCode];
        break;
    case 3:
        retval = threeBitMapping[ddcTypeCode];
        break;
    default:
        ELOG << "Don't know how to handle " << nTypeBits <<
                "-bit DDC type code!";
        retval = DDCUNDEFINED;
    }
    
    return retval;
}

//////////////////////////////////////////////////////////////////////
void
p7142sd3c::loadFreeRun() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (isSimulating())
        return;

    // get the transceiver control register
    uint32_t tcreg;
    P7142_REG_READ(_BAR2Base + TRANS_CNTRL, tcreg);

    // set the free run bit as specified by _freerun
    if (_freeRun) {
        // set free run
    	P7142_REG_WRITE(_BAR2Base + TRANS_CNTRL, tcreg | TRANS_FREE_RUN);
        usleep(p7142::P7142_IOCTLSLEEPUS);
    } else {
        // clear free run
    	P7142_REG_WRITE(_BAR2Base + TRANS_CNTRL, tcreg & ~TRANS_FREE_RUN);
        usleep(p7142::P7142_IOCTLSLEEPUS);
    }

}

/////////////////////////////////////////////////////////////////////////
bool
p7142sd3c::initTimers(bool isStart) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_simulate)
        return true;

    //    This section initializes the timers.

    int periodCount = 0; // Period Count for all Timers
    int PrtScheme = 0; // PRT Scheme for all Timers

    // Calculate the period and PRT Scheme for dual prt or single prt
    // Note: _prtCounts and _prt2Counts are expressed in SD3C Timer counts!

    int X, Y;
    double prt_ms, prt2_ms;

    if (isStart) {
      DLOG << "===================== initTimers =======================";
    } else {
      DLOG << "===================== stopping Timers =======================";
    }

    if (_staggeredPrt == true) {
        // dual prt
        prt_ms = countsToTime(_prtCounts) * 1000;
        prt2_ms = countsToTime(_prt2Counts) * 1000;
        double prt_ratio = prt2_ms / prt_ms;
        int rounded_ratio = (int) (prt_ratio);
        periodCount =
          timeToCounts(prt_ms * (prt_ratio - rounded_ratio) / rounded_ratio * 0.001);

        double xx = rounded_ratio / (prt_ratio - rounded_ratio);
        double yy = xx * prt_ratio;
        
        X = (int) (rounded_ratio / (prt_ratio - rounded_ratio) + 0.5);
        Y = (int) (X * prt_ratio + 0.5);

        PrtScheme = (Y << 4) | X;

        char text[128];

        if (isStart) {
          DLOG << "_prtCounts: " << _prtCounts;
          DLOG << "_prt2Counts: " << _prt2Counts;
          sprintf(text, "  prt_ms:    %.12f", prt_ms);
          DLOG << text;
          sprintf(text, "  prt2_ms:   %.12f", prt2_ms);
          DLOG << text;
          sprintf(text, "  prt_ratio: %.12f", prt_ratio);
          DLOG << text;
          DLOG << "Staggered periodCount: " << periodCount;
          DLOG << "Staggered, PrtScheme: " << PrtScheme;
          DLOG << "  rounded_ratio: " << rounded_ratio;
          sprintf(text, "  xx, yy: %.12f, %.12f", xx, yy);
          DLOG << text;
          DLOG << "  X, Y: " << X << ", " << Y;
        }
          
    } else {

        // Single prt
    	// PRT must be integral multiple of pulsewidth !
        periodCount = _prtCounts;
        PrtScheme = 0x0000;

        if (isStart) {
          DLOG << "Non-staggered periodCount is " << periodCount;
        }

    }
    if (isStart) {
      DLOG << "========================================================";
    }
    
    if (isStart && periodCount > 65535) {
      ELOG << "********************** ERROR ****************************";
      ELOG << "==>> initTimers() error, period count > 65535: " << periodCount;
      ELOG << "     Cannot start timers";
      ELOG << " ";
      ELOG << "Consider increasing the sd3cTimerDivisor used when instantiating";
      ELOG << "the p7142sd3c object. That will decrease the resolution of the";
      ELOG << "SD3C timers, but will increase the maximum period that can be";
      ELOG << "set for the timers (e.g., will allow a longer PRT).";
      ELOG << "*********************************************************";
      return false;
    }

    // Turn on write strobes
    P7142_REG_WRITE(_BAR2Base + MT_WR, WRITE_ON);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    // Set timer delay and width for each SD3C timer individually
    DLOG << "ADC freq.: " << 1.0e-6 * adcFrequency() << " MHz, " <<
            "SD3C freq.:" << 1.0e-6 * _sd3cFrequency() <<
            " MHz (using ADC divisor " << _sd3cTimerDivisor << ")";
    for (unsigned int i = 0; i < N_SD3C_TIMERS; i++) {
        DLOG << "Timer " << i << " (" << timerName(i) <<
                "): delay " << countsToTime(timerDelay(i)) << " s (" <<
                timerDelay(i) << "), width " << countsToTime(timerWidth(i)) <<
                " s (" << timerWidth(i) << ")" <<
                (timerInvert(i)? ", inverted" : "") <<
                (timerEnabled(i)? ", enabled" : ", disabled");

        // Control Register
        // Address
        P7142_REG_WRITE(_BAR2Base + MT_ADDR, CONTROL_REG | SD3C_TIMER_BITS[i]);
        usleep(p7142::P7142_IOCTLSLEEPUS);
        // Data
        P7142_REG_WRITE(_BAR2Base + MT_DATA, _timerControlRegData(i));
        usleep(p7142::P7142_IOCTLSLEEPUS);

        // Delay Register
        // Address
        P7142_REG_WRITE(_BAR2Base + MT_ADDR, DELAY_REG | SD3C_TIMER_BITS[i]);
        usleep(p7142::P7142_IOCTLSLEEPUS);
        // Data
        P7142_REG_WRITE(_BAR2Base + MT_DATA, timerDelay(i));
        usleep(p7142::P7142_IOCTLSLEEPUS);

        // Timer Width Register
        // Address
        P7142_REG_WRITE(_BAR2Base + MT_ADDR, WIDTH_REG | SD3C_TIMER_BITS[i]);
        usleep(p7142::P7142_IOCTLSLEEPUS);
        // Data
        P7142_REG_WRITE(_BAR2Base + MT_DATA, timerWidth(i));
        usleep(p7142::P7142_IOCTLSLEEPUS);
    }

    // All timers have identical configuration for period and multiple prt

    // Period Register
    // Address
    P7142_REG_WRITE(_BAR2Base + MT_ADDR, PERIOD_REG | ALL_SD3C_TIMER_BITS);
    usleep(p7142::P7142_IOCTLSLEEPUS);
    // Data
    P7142_REG_WRITE(_BAR2Base + MT_DATA, periodCount);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    //Multiple PRT Register
    // Address
    P7142_REG_WRITE(_BAR2Base + MT_ADDR, PRT_REG | ALL_SD3C_TIMER_BITS);
    usleep(p7142::P7142_IOCTLSLEEPUS);
    // Data: Mult PRT Valu Timer 0
    P7142_REG_WRITE(_BAR2Base + MT_DATA, PrtScheme);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    // Turn off Write Strobes
    P7142_REG_WRITE(_BAR2Base + MT_WR, WRITE_OFF);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    return true;

}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::dataRate() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    int rate = 0;

    switch (_mode) {
    case MODE_FREERUN:
        // two bytes of I and two bytes of Q for each range gate
        rate = _gates*4;
        break;
    case MODE_PULSETAG:
        // pulse tagger
        // there is a four byte sync word and a four byte pulse tag
        // at the beginning of each pulse. There are two bytes for each
        // I and each Q for each range gate.
        rate = (int)(_prf * (4 + 4 + _gates*4));
        break;
    case MODE_CI:
        // coherent integration
        // there is a 16 byte tag at the beginning of each pulse. Each pulse
        // returns a set of even I's and Q's, and a set of odd I's and Q's. The
        // even and odd pulses are separated by the prt, and so taken together they
        // run at half the prf. Each I and Q for a gate is 32 bits (wider than the
        // non-CI mode because they are sums of 16 bit numbers), so there are 8 bytes
        // per gate for even and 8 bytes per gate for odd pulses. Nsum is the number of
    	// pulses that were added in both the even and odd accumulators.
        rate = (int)((_prf/_nsum)*(16+_gates*8*2));
        break;
    case MODE_CI_RIM:
        // coherent integration with range imaging
        // there is a 48 byte tag at the beginning of each pulse. Each pulse
        // returns a set of even I's and Q's, and a set of odd I's and Q's, for
    	// four frequencies. The even and odd pulses are separated
    	// by the prt, and so taken together they
        // run at half the prf. Each I and Q for a gate is 32 bits (wider than the
        // non-CI mode because they are sums of 16 bit numbers), so there are 8 bytes
        // per gate for even and 8 bytes per gate for odd pulses. Nsum is the number of
    	// pulses that were added in both the even and odd accumulators.
        rate = (int)((_prf/_nsum)*(48+ 4*_gates*8*2));
        break;
    }

    return rate;
}

//////////////////////////////////////////////////////////////////////
int64_t p7142sd3c::pulseAtTime(ptime time) const {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);
    
    // First get the time since transmit start, in seconds
    double timeSinceStart = 1.0e-6 * (time - _radarStartTime).total_microseconds();
    
    // Now figure out the pulse number. Note that this should work even for
    // times before the radar start time (yielding negative pulse number).
    int64_t pulseNum = 0;
    if (_staggeredPrt) {
        // First count the complete pairs of PRT1 and PRT2
        double pairTime = _prt + _prt2;
        pulseNum = 2 * int64_t(timeSinceStart / pairTime);
        // Then work with the remaining time that's a fraction of (PRT1 + PRT2)
        double remainingTime = fmod(timeSinceStart, pairTime);
        double absRemainingTime = fabs(remainingTime);
        int sign = (remainingTime < 0) ? -1 : 1;
        
        if (absRemainingTime > (_prt + 0.5 * _prt2)) {
            pulseNum += sign * 2;
        } else if (absRemainingTime > (0.5 * _prt)) {
            // If the remainder is greater than halfway between 0 and PRT1,
            // add 1 to the pulse count
            pulseNum += sign;
        }
    } else {
        pulseNum = int64_t(timeSinceStart / _prt);
        double remainingTime = fmod(timeSinceStart, _prt);
        int sign = (remainingTime < 0) ? -1 : 1;
        // If the remainder is more than 1/2 a PRT, add another pulse
        if (fabs(remainingTime) > (0.5 * _prt)) {
            pulseNum += sign;
        }
    }
    
    /// pulse numbers are 1-based
    /// so add 1 to the computed number
    return(pulseNum + 1);
}

//////////////////////////////////////////////////////////////////////
std::string p7142sd3c::ddcTypeName(DDCDECIMATETYPE type) {
    switch (type) {
    case DDC10DECIMATE:
        return std::string("DDC10DECIMATE");
    case DDC8DECIMATE:
        return std::string("DDC8DECIMATE");
    case DDC4DECIMATE:
        return std::string("DDC4DECIMATE");
    case BURST:
        return std::string("BURST");
    case DDC6DECIMATE:
        return std::string("DDC6DECIMATE");
    default:
        return std::string("Unknown");
    }
}

//////////////////////////////////////////////////////////////////////
std::string p7142sd3c::ddcTypeName() const
{
    return ddcTypeName(_ddcType);
}

//////////////////////////////////////////////////////////////////////
uint16_t p7142sd3c::ddcDecimation() const
{
    switch (_ddcType) {
    case DDC10DECIMATE:
        return(10);
    case DDC8DECIMATE:
        return(8);
    case DDC6DECIMATE:
        return(6);
    case DDC4DECIMATE:
        return(4);
    case BURST:
        return(1);
    default:
        return(0);
    }
}

//////////////////////////////////////////////////////////////////////
double p7142sd3c::txPulseWidth() const
{
	return countsToTime(txPulseWidthCounts());
}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::txPulseWidthCounts() const
{
	return timerWidth(TX_PULSE_TIMER);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::setGPTimer0(double delay, double width, bool invert)
{
    setTimer(GP_TIMER_0, timeToCounts(delay), timeToCounts(width), true, invert);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::setGPTimer1(double delay, double width, bool invert)
{
    setTimer(GP_TIMER_1, timeToCounts(delay), timeToCounts(width), true, invert);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::setGPTimer2(double delay, double width, bool invert)
{
    setTimer(GP_TIMER_2, timeToCounts(delay), timeToCounts(width), true, invert);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::setGPTimer3(double delay, double width, bool invert)
{
    setTimer(GP_TIMER_3, timeToCounts(delay), timeToCounts(width), true, invert);
}

//////////////////////////////////////////////////////////////////////
unsigned int p7142sd3c::gates() const
{
	return _gates;
}

//////////////////////////////////////////////////////////////////////
unsigned int p7142sd3c::nsum() const
{
	return _nsum;
}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::codeLength() const
{
	return _codeLength;
}

//////////////////////////////////////////////////////////////////////
const std::string
p7142sd3c::timerName(int timerNdx) const {
    switch (timerNdx) {
    case MASTER_SYNC_TIMER:
        return("MASTER_SYNC_TIMER");
        break;
    case RX_01_TIMER:
        return("RX_01_TIMER");
        break;
    case TX_PULSE_TIMER:
        return("TX_PULSE_TIMER");
        break;
    case GP_TIMER_0:
        return("GP_TIMER_0");
        break;
    case RX_23_TIMER:
        return("RX_23_TIMER");
        break;
    case GP_TIMER_1:
        return("GP_TIMER_1");
        break;
    case GP_TIMER_2:
        return("GP_TIMER_2");
        break;
    case GP_TIMER_3:
        return("GP_TIMER_3");
        break;
    default:
        // fall through
        break;
    }
    return("UNKNOWN_TIMER");
}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::timerDelay(int timerNdx) const {
    return(_timerConfigs[timerNdx].delay());
}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::timerWidth(int timerNdx) const {
    return(_timerConfigs[timerNdx].width());
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3c::timerInvert(int timerNdx) const {
    return(_timerConfigs[timerNdx].invert());
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3c::timerEnabled(int timerNdx) const {
    return(_timerConfigs[timerNdx].enabled());
}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::sd3cRev() const {
        return(_sd3cRev);
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3c::motorZeroPositionSet() const {
        return(_motorZeroPositionSet);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::zeroMotorCounts() {
    // Bit zero of the TTL_OUT1 register enables zeroing of the counts for
    // all motors being monitored. Set the bit to 1 to clear the counts, then
    // set the bit back to 0 to resume normal quadrature counting.
    ILOG << "Zeroing motor counts";
    uint16_t regVal = TTLIn();
    TTLOut(regVal | 0x1);       // set the bit
    usleep(500);                // leave the bit high for 500 us
    TTLOut(regVal & 0xfffe);    // clear the bit

    // Note that motor zero position has been set
    _motorZeroPositionSet = true;
}

uint16_t p7142sd3c::_timerControlRegData(int timerNdx) const {
    uint16_t controlRegData = 0;

    // Set the enabled/disabled state
    controlRegData |=
            _timerConfigs[timerNdx].enabled() ? TIMER_ENABLED : TIMER_DISABLED;

    // Set the normal/inverted state
    controlRegData |=
            _timerConfigs[timerNdx].invert() ? TIMER_NEG : TIMER_POS;

    // Set the clock divider
    //
    // WARNING: The control register clock divider macros in DDCregisters.h
    // are named w.r.t. the base SD3C clock frequency, which is
    // ADC_clock_frequency/2. Our _sd3cTimerDivisor member describes division
    // w.r.t. ADC_clock frequency. Hence the apparent factor-of-two discrepancy
    // in the assignments below...
    switch (_sd3cTimerDivisor) {
    case 2:
        controlRegData |= CLK_DIV1;
        break;
    case 4:
        controlRegData |= CLK_DIV2;
        break;
    case 8:
        controlRegData |= CLK_DIV4;
        break;
    case 16:
        controlRegData |= CLK_DIV8;
        break;
    default:
        // This should have been caught in the constructor...
        ELOG << "BUG: unsupported SD3C timer divisor " << _sd3cTimerDivisor;
        abort();
        break;
    }

    return(controlRegData);
}

double
p7142sd3c::DefaultAdcFrequency(DDCDECIMATETYPE ddcType) {
    // Return the default ADC clock rate based on DDC type
    switch (ddcType) {
      case DDC10DECIMATE:
        return(100.0e6);    // DDC10 -> 100 MHz
        break;
      case DDC8DECIMATE:
        return(125.0e6);    // DDC8 -> 125 MHz
        break;
      case DDC6DECIMATE:
        return(80.0e6);     // DDC6 -> 80 MHz
        break;
      case DDC4DECIMATE:
        return(48.0e6);     // DDC4 -> 48 MHz
        break;
      case BURST:
        return(100.0e6);
        break;
      default:
        ELOG << "Unhandled DDC type " << ddcTypeName(ddcType) << "!";
    }
    return(NAN);
}

} // end namespace Pentek
