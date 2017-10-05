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
#include "p7142sd3cDn.h"
#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"
#include "FilterSpec.h"
#include <sys/ioctl.h>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

#include <logx/Logging.h>
LOGGING("p7142sd3cDn");

using namespace boost::posix_time;

namespace Pentek {

const double p7142sd3cDn::SPEED_OF_LIGHT = 2.99792458e8;      // m s-1

////////////////////////////////////////////////////////////////////////////////
/// @brief Return the least common multiple of the two given integers.
/// @param a the first integer for comparison
/// @param b the second integer for comparison
/// @return the least common multiple of a and b
static int
leastCommonMultiple(int a, int b) {
    // First find the greatest common divisor of a and b.
    int gcd;
    int c = a;
    int d = b;
    while (true) {
        if (c == 0) {
            gcd = d;
            break;
        }
        d %= c;
        if (d == 0) {
            gcd = c;
            break;
        }
        c %= d;
    }

    // Now get LCM of a and b
    int lcm = gcd ? ((a * b) / gcd) : 0;
    return(lcm);
}

////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn::p7142sd3cDn(p7142sd3c * p7142sd3cPtr, int chanId, 
        uint32_t dmaDescSize, bool isBurst, int tsLength, double rx_delay,
        double rx_pulsewidth, std::string gaussianFile, std::string kaiserFile,
        int simWaveLength,
        bool internalClock /* = false */,
        bool abortOnError /* = true */) :
        p7142Dn(p7142sd3cPtr, 
                chanId, 
                dmaDescSize,
                1, 
                simWaveLength,
                p7142sd3cPtr->nsum() > 1,
                internalClock,
                abortOnError),
        _sd3c(*p7142sd3cPtr),
        _isBurst(isBurst),
        _tsLength(tsLength),
        _gaussianFile(gaussianFile), 
        _kaiserFile(kaiserFile),
        _lastPulse(0),
        _nPulsesSinceStart(0),
        _droppedPulses(0),
        _syncErrors(0),
        _firstRawBeam(true),
        _firstBeam(true),
        _dataInterruptPeriod(0.0)
{

    _constructorOk = true;

    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    // Get gate count and coherent integration sum count from our card
    _gates = _sd3c.gates();
    _nsum = _sd3c.nsum();
    uint16_t sd3cTimerDivisor = _sd3c._sd3cTimerDivisor;
    
    // log startup params in debug mode

    DLOG << "+++++++++++++++++++++++++++++";
    DLOG << "p7142sd3cDn constructor";
    DLOG << "  abortOnError: " << _abortOnError;
    DLOG << "  cardIndex: " << _p7142.getCardIndex();
    DLOG << "  chanId: " << chanId;
    DLOG << "  isBurst: " << isBurst;
    DLOG << "  tsLength: " << tsLength;
    DLOG << "  rx_delay: " << rx_delay;
    DLOG << "  rx_pulsewidth: " << rx_pulsewidth;
    DLOG << "  gaussianFile: " << gaussianFile;
    DLOG << "  kaiserFile: " << kaiserFile;
    DLOG << "  simWaveLength: " << simWaveLength;
    DLOG << "  internalClock: " << internalClock;
    DLOG << "  gates: " << _gates;
    DLOG << "  nsum: " << _nsum;
    DLOG << "++++++++++++++++++++++++++++";

    // Gating quantum period in ADC clock counts. SD3C decimates by its
    // DDC-specific factor before delivering data, so the DDC writes decimated
    // data to the output FIFO at this interval in ADC clock counts.
    // Note that burst channels always use a decimation factor of 2.
    uint16_t gateQuantumAdcCounts = _isBurst ? 2 : _sd3c.ddcDecimation();

    // Make sure the time per gate is a non-zero multiple of the gating quantum
    // period. We work in ADC counts here because timing for an individual
    // gate is constrained by the ADC clock, but NOT by the SD3C timer
    // clock (which is a factor of 2 or more slower).
    int32_t oneGateAdcCounts = _sd3c._timeToAdcCounts(rx_pulsewidth);
    if ((oneGateAdcCounts / gateQuantumAdcCounts) == 0 ||
            (oneGateAdcCounts % gateQuantumAdcCounts) != 0) {
        ELOG << (_isBurst ? "burst " : "") <<
                "rx_pulsewidth (digitizer_sample_width) must be a multiple of " <<
                1.0e9 * gateQuantumAdcCounts / _sd3c.adcFrequency() <<
                " ns for " << _sd3c.ddcTypeName();
        if (_abortOnError) {
            abort();
        } else {
            _constructorOk = false;
        }
    }

    // Burst channel is funky: we sample as fast as possible for a period of
    // the rx_pulsewidth (i.e., oneGateAdcCounts). Adjust _gates now if
    // necessary to reflect the number of samples we'll actually get.
    if (_isBurst) {
        _gates = oneGateAdcCounts / gateQuantumAdcCounts;
    }

    // Get the interval between SD3C timer counts which align to a gate
    // boundary.
    int gateAlignedSd3cInterval =
            leastCommonMultiple(oneGateAdcCounts, sd3cTimerDivisor) / sd3cTimerDivisor;
    DLOG << "PRT (and PRT2) must be a multiple of " <<
            1.0e9 * _sd3c.countsToTime(gateAlignedSd3cInterval) <<
            " ns for the given ADC clock and timer divisor values.";

    // Get PRTs in rounded SD3C timer counts and verify that they align on
    // both an SD3C timer clock boundary and on a gate boundary.
    uint32_t prtCounts = _sd3c.prtCounts();
    uint32_t remainderCounts = prtCounts % gateAlignedSd3cInterval;
    uint32_t prt2Counts = _sd3c.prt2Counts();
    uint32_t remainder2Counts = prt2Counts % gateAlignedSd3cInterval;
    if (remainderCounts != 0 || remainder2Counts != 0) {
        ELOG << "PRT (and PRT2) must be a multiple of " <<
                1.0e9 * _sd3c.countsToTime(gateAlignedSd3cInterval) <<
                " ns for the given ADC clock and timer divisor values.";
        if (_abortOnError) {
            abort();
        } else {
            _constructorOk = false;
        }
    }

    // Get the receiver gate timer delay (rcvr_gate0_delay) in rounded SD3C
    // timer counts, then adjust the count up if necessary to make sure the
    // delay ends at a gate boundary.
    int32_t rxGateDelayCounts = _sd3c.timeToCounts(rx_delay);
    remainderCounts = rxGateDelayCounts % gateAlignedSd3cInterval;
    if (remainderCounts != 0) {
        rxGateDelayCounts += gateAlignedSd3cInterval - remainderCounts;
        WLOG << "rx_gate0_delay adjusted up for alignment: requested delay " <<
                1.0e9 * rx_delay << " ns -> actual delay " <<
                1.0e9 * _sd3c.countsToTime(rxGateDelayCounts) << " ns";
    }

    // Calculate the receiver gate timer width in SD3C timer counts. We
    // get the needed time in ADC counts and divide down to SD3C counts.
    // If the ADC counts do not divide evenly into the SD3C counts,
    // adjust the next SD3C count.
    uint32_t rxGateWidthAdcCounts =
            _isBurst ? oneGateAdcCounts : _gates * oneGateAdcCounts;
    uint32_t rxGateWidthCounts = rxGateWidthAdcCounts / sd3cTimerDivisor; // ADC counts -> SD3C counts
    uint32_t remainderAdcCounts = rxGateWidthAdcCounts % sd3cTimerDivisor;
    if (remainderAdcCounts != 0) {
        rxGateWidthCounts++;
    }

    // Make sure the PRT is long enough for the receiver delay plus the
    // requested number of gates.
    if (! _isBurst) {
        uint32_t endOfRxGateTimer = rxGateDelayCounts + rxGateWidthCounts;
        if ((prtCounts < endOfRxGateTimer) ||
            (prt2Counts && prt2Counts < endOfRxGateTimer)) {
            ELOG << "PRT(s) too short for rxGateDelay of " <<
                    1.0e6 * _sd3c.countsToTime(rxGateDelayCounts) <<
                    " us + " << _gates << " gates @ " <<
                    1.0e6 * oneGateAdcCounts / _sd3c.adcFrequency() <<
                    " us/gate";
            if (_abortOnError) {
                abort();
            } else {
                _constructorOk = false;
            }
        }
    }

    // Set the rx gating timer. 
    // Note that Channels 0 and 1 share RX_01_TIMER, and channels 2 and 3 
    // share RX_23_TIMER.
    // For a burst sampling channel, take as many gates as the clock allows 
    // over the given pulse width, and set _gates to the correct value here.
    p7142sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
            p7142sd3c::RX_01_TIMER : p7142sd3c::RX_23_TIMER;
    _sd3c.setTimer(rxTimerNdx, rxGateDelayCounts, rxGateWidthCounts);
    
    /// @todo Estimate the period between data-available interrupts based on the
    /// configured interrupt buffer length. This is a good first-order estimate
    /// of maximum data latency time for the channel.
    /// @todo Configure the channel intbufsize for roughly 10 Hz interrupts
    /// (and bufsize to ~2*intbufsize)
    
    int interruptBytes;
    if (isSimulating()) {
    	interruptBytes =  32768;
    } else {
    	interruptBytes = 65536; /// @todo This scheme needs to be revised once we get the windriver DMA working.
    }

    double chanDataRate = (4 * _gates) / _sd3c.prt();   /// @todo this only works for single PRT
    _dataInterruptPeriod = interruptBytes / chanDataRate;
    if (p7142sd3cPtr->nsum() > 1) {
    	_dataInterruptPeriod /= (p7142sd3cPtr->nsum()/2);
    }

    // Warn if data latency is greater than 1 second, and bail completely if
    // it's greater than 5 seconds.
    if (_dataInterruptPeriod > 1.0) {
        ELOG << "interruptBytes " << interruptBytes;
        ELOG << "chanDataRate " << chanDataRate;
        ELOG << "Warning: Estimated max data latency for channel " << 
        _chanId << " is " << _dataInterruptPeriod << " s!";
    }

    if (_dataInterruptPeriod > 5.0) {
        if (_abortOnError) {
          abort();
        }
        _constructorOk = false;
    }
    
    // initialize the buffering scheme.
    initBuffer();

    if (isSimulating()) {
        return;
    }

    // Set the bypass divider (decimation) for our receiver channel
    // ** This establishes the gate width in the downconverter. **
    int bypassOk = _isBurst ?
            setBypassDivider(2) : setBypassDivider(oneGateAdcCounts);
    if (!bypassOk) {
        ELOG << "Failed to set decimation for channel " << _chanId;
        if (_abortOnError) {
          abort();
        }
        _constructorOk = false;
    }
    DLOG << "bypass decim: " << bypassDivider();
    
    // configure DDC in FPGA
    if (!config()) {
      DLOG << "error initializing filters";
      if (_abortOnError) {
        abort();
      }
      _constructorOk = false;
    }

}

////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn::~p7142sd3cDn() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    delete [] _buf;
}

////////////////////////////////////////////////////////////////////////////////
std::string p7142sd3cDn::ddcTypeName() const {
    return p7142sd3c::ddcTypeName(_sd3c.ddcType());
}

////////////////////////////////////////////////////////////////////////////////
double p7142sd3cDn::rcvrPulseWidth() const {
    // Note that Channels 0 and 1 share RX_01_TIMER, and channels 2 and 3 
    // share RX_23_TIMER.
    p7142sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
            p7142sd3c::RX_01_TIMER : p7142sd3c::RX_23_TIMER;
    return(_sd3c.countsToTime(_sd3c.timerWidth(rxTimerNdx)) / _gates);
}

////////////////////////////////////////////////////////////////////////////////
double p7142sd3cDn::rcvrFirstGateDelay() const {
    int txDelayCounts = _sd3c.timerDelay(p7142sd3c::TX_PULSE_TIMER);
    // Note that Channels 0 and 1 share RX_01_TIMER, and channels 2 and 3 
    // share RX_23_TIMER.
    p7142sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
            p7142sd3c::RX_01_TIMER : p7142sd3c::RX_23_TIMER;
    int rxDelayCounts = _sd3c.timerDelay(rxTimerNdx);

    return(_sd3c.countsToTime(rxDelayCounts - txDelayCounts));
}

////////////////////////////////////////////////////////////////////////////////
double p7142sd3cDn::gateSpacing() const {
    return(0.5 * SPEED_OF_LIGHT * rcvrPulseWidth());
}

////////////////////////////////////////////////////////////////////////////////
bool p7142sd3cDn::config() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // configure the fifo
    fifoConfig();

    // Is coherent integration enabled?
    DLOG << "coherent integration is " <<
          (_nsum > 1 ? "enabled" : "disabled");

    // set up the filters. Will do nothing if either of
    // the filter file paths is empty or if this is a burst channel
    bool filterError = filterSetup();
    if (filterError) {
      DLOG << "p7142sd3cDn::config() filterSetup() failed";
      return false;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3cDn::loadFilters(FilterSpec& gaussian, FilterSpec& kaiser) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return true;
    
    int ddcSelect = _chanId << 14;

    // program the kaiser coefficients

    DLOG << "Loading kaiser filter: " << kaiser.name();

    std::vector<unsigned int> kaiserReadBacks;
    for (size_t ii = 0; ii < kaiser.size(); ii++) {

        // Set up to write this coefficient
        int ramAddr = 0;
        int ramSelect = 0;
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC10DECIMATE:
            ramAddr = ii / 10;
            ramSelect = (ii % 10) << 4;
            break;
        case p7142sd3c::DDC8DECIMATE:
            ramAddr = ii / 8;
            ramSelect = (ii % 8) << 4;
            break;
        case p7142sd3c::DDC6DECIMATE:
            ramAddr = ii / 6;
            ramSelect = (ii % 6) << 4;
            break;
        case p7142sd3c::DDC4DECIMATE:
            ramAddr = ii / 4;
            ramSelect = (ii % 4) << 4;
            break;
        case p7142sd3c::BURST:   // Burst mode uses no filters
            break;
        case p7142sd3c::DDCUNDEFINED:
        default:
            ELOG << "DDC type " << ddcTypeName() << " not handled at " <<
                    __FUNCTION__ << ":" << __LINE__;
            break;
        }
        
        // Try up to a few times to program this filter coefficient and
        // read it back successfully.

        unsigned int kaiserReadBack = 0;
        for (int iattempt = 0; iattempt < 5; iattempt++) {

            P7142_REG_WRITE(_sd3c._BAR2Base + KAISER_ADDR,
                            ddcSelect | DDC_STOP | ramSelect | ramAddr);
            usleep(1);

            // write the value
            // LS word first
            P7142_REG_WRITE(_sd3c._BAR2Base + KAISER_DATA_LSW, kaiser[ii] & 0xFFFF);
            usleep(1);
    
            // then the MS word -- since coefficients are 18 bits and FPGA 
            // registers are 16 bits!
            P7142_REG_WRITE(_sd3c._BAR2Base + KAISER_DATA_MSW,
                    (kaiser[ii] >> 16) & 0x3);
            usleep(1);
    
            // latch coefficient
            P7142_REG_WRITE(_sd3c._BAR2Base + KAISER_WR, 0x1);
            usleep(1);
    
            // disable writing (kaiser readback only succeeds if we do this)
            P7142_REG_WRITE(_sd3c._BAR2Base + KAISER_WR, 0x0);
            usleep(1);
    
            // read back the programmed value; we need to do this in two words 
            // as above.
            uint32_t kaiser_lsw;
            uint32_t kaiser_msw;
            P7142_REG_READ(_sd3c._BAR2Base + KAISER_READ_LSW, kaiser_lsw);
            P7142_REG_READ(_sd3c._BAR2Base + KAISER_READ_MSW, kaiser_msw);
            kaiserReadBack = kaiser_msw << 16 | kaiser_lsw;
            
            if (kaiserReadBack == kaiser[ii]) {
              break;
            }
            
        } // iattempt
        
        kaiserReadBacks.push_back(kaiserReadBack);

    } // ii

    // check for failure in loading kaiser
    
    bool kaiserFailed = false;
    for (size_t ii = 0; ii < kaiserReadBacks.size(); ii++) {
      if (kaiserReadBacks[ii] != kaiser[ii]) {
        kaiserFailed = true;
        break;
      }
    }

    if (kaiserFailed) {
      ELOG << "ERROR - p7142sd3cDn::loadFilters()";
      ELOG << "FAILED to load and readback Kaiser filter";
      ELOG << "  path: " << kaiser.path();
      ELOG << "  name: " << kaiser.name();
      ELOG << "  size: " << kaiser.size();
      ELOG << "  isSymmetric: " << kaiser.isSymmetric();
      for (size_t ii = 0; ii < kaiserReadBacks.size(); ii++) {
        char msg[1024];
        sprintf(msg, "    index, val, readback: %3d %10d %10d",
                (int) ii, (int) kaiser[ii], (int) kaiserReadBacks[ii]);
        ELOG << msg;
      } // ii
    } else {
      DLOG << "SUCCESS loading kaiser filter - p7142sd3cDn::loadFilters()";
    }

    // program the gaussian coefficients
    // Note that the DDC select is accomplished in the filter coefficient
    // address register, which was done during the previous kaiser filter load.

    DLOG << "Loading gaussian filter: " << gaussian.name();
    DLOG << "        gaussian.size(): " << gaussian.size();

    std::vector<unsigned int> gaussianReadBacks;
    for (size_t ii = 0; ii < gaussian.size(); ii++) {

        // Set up to write this coefficient
        int ramAddr = 0;
        int ramSelect = 0;
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC10DECIMATE:
            ramAddr = ii % 10;
            ramSelect = (ii / 10) << 4;
            break;
        case p7142sd3c::DDC8DECIMATE:
            ramAddr = ii % 8;
            ramSelect = (ii / 8) << 4;
            break;    
        case p7142sd3c::DDC6DECIMATE:
            ramAddr = ii % 12;
            ramSelect = (ii / 12) << 4;
            break;    
        case p7142sd3c::DDC4DECIMATE:
            ramAddr = ii % 12;
            ramSelect = (ii / 12) << 4;
            break;
        case p7142sd3c::BURST:   // Burst mode uses no filters
            break;    
        case p7142sd3c::DDCUNDEFINED:
        default:
            ELOG << "DDC type " << ddcTypeName() << " not handled at " <<
                    __FUNCTION__ << ":" << __LINE__;
            break;
        }

        /// @todo early versions of the gaussian filter programming required
        /// the ds select bits to be set in the gaussian address register.
        /// We can take this out when we get a working bitstream with this
        /// fixed

        // Try up to a few times to program this filter coefficient and
        // read it back successfully.
        unsigned int gaussianReadBack = 0;
        for (int iattempt = 0; iattempt < 5; iattempt++) {

            // set the address
            P7142_REG_WRITE(_sd3c._BAR2Base + GAUSSIAN_ADDR,
                    ddcSelect | ramSelect | ramAddr);
            usleep(1);
    
            // write the value
            // LS word first
            P7142_REG_WRITE(_sd3c._BAR2Base + GAUSSIAN_DATA_LSW,
                    gaussian[ii] & 0xFFFF);
            usleep(1);
    
            // then the MS word -- since coefficients are 18 bits and FPGA 
            // registers are 16 bits!
            P7142_REG_WRITE(_sd3c._BAR2Base + GAUSSIAN_DATA_MSW,
                    (gaussian[ii] >> 16) & 0x3);
            usleep(1);
    
            // latch coefficient
            P7142_REG_WRITE(_sd3c._BAR2Base + GAUSSIAN_WR, 0x1);
            usleep(1);
    
            // disable writing (gaussian readback only succeeds if we do this)
            P7142_REG_WRITE(_sd3c._BAR2Base + GAUSSIAN_WR, 0x0);
            usleep(1);
    
            // read back the programmed value; we need to do this in two words 
            // as above.
            uint32_t gaussian_lsw;
            uint32_t gaussian_msw;
            P7142_REG_READ(_sd3c._BAR2Base + GAUSSIAN_READ_LSW, gaussian_lsw);
            P7142_REG_READ(_sd3c._BAR2Base + GAUSSIAN_READ_MSW, gaussian_msw);
            gaussianReadBack = gaussian_msw << 16 | gaussian_lsw;

            if (gaussianReadBack == gaussian[ii]) {
              break;
            }

        } // iattempt

        gaussianReadBacks.push_back(gaussianReadBack);
        
    } // ii

    // check for failure in loading gaussian
    
    bool gaussianFailed = false;
    for (size_t ii = 0; ii < gaussianReadBacks.size(); ii++) {
      if (gaussianReadBacks[ii] != gaussian[ii]) {
        gaussianFailed = true;
        break;
      }
    }

    if (gaussianFailed) {
      ELOG << "ERROR - p7142sd3cDn::loadFilters()";
      ELOG << "FAILED to load and readback Gaussian filter";
      ELOG << "  path: " << gaussian.path();
      ELOG << "  name: " << gaussian.name();
      ELOG << "  size: " << gaussian.size();
      ELOG << "  isSymmetric: " << gaussian.isSymmetric();
      for (size_t ii = 0; ii < gaussianReadBacks.size(); ii++) {
        char msg[1024];
        sprintf(msg, "    index, val, readback: %3d %10d %10d",
                (int) ii, (int) gaussian[ii], (int) gaussianReadBacks[ii]);
        DLOG << msg;
      } // ii
    } else {
      DLOG << "SUCCESS loading gaussian filter - p7142sd3cDn::loadFilters()";
    }

    return !kaiserFailed && !gaussianFailed;

}

////////////////////////////////////////////////////////////////////////
int p7142sd3cDn::filterSetup() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // No filters if this is a burst sampling channel
    if (_isBurst)
        return 0;

    // Transmit pulsewidth rounded to the nearest integer nanosecond
    long iPulsewidth_ns = lround(1.0e9 *
            _sd3c.countsToTime(_sd3c.timerWidth(p7142sd3c::TX_PULSE_TIMER)));
    DLOG << "filterSetup(): " << _sd3c.ddcTypeName() << " iPulsewidth_ns is " <<
            iPulsewidth_ns << " ns";

    // get the gaussian filter coefficients.
    FilterSpec gaussian;
    if (_gaussianFile.size() != 0) {
        FilterSpec g(_gaussianFile);
        if (!g.ok()) {
            ELOG << "Incorrect or unaccessible filter definition: "
                    << _gaussianFile;
            return -1;
        } else {
            gaussian = g;
        }
    } else {
        std::string gaussianFilterName("");
        BuiltinGaussian builtins;

        // Choose the correct builtin Gaussian filter coefficient set.
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC8DECIMATE: {
            switch (iPulsewidth_ns) {
            case 256:
                gaussianFilterName = "ddc8_0_2";
                break;
            case 384:
                gaussianFilterName = "ddc8_0_3";
                break;
            case 512:
                gaussianFilterName = "ddc8_0_5";
                break;
            case 640:
                gaussianFilterName = "ddc8_0_6";
                break;
            case 768:
                gaussianFilterName = "ddc8_0_7";
                break;
            case 896:
                gaussianFilterName = "ddc8_0_8";
                break;
            case 1024:
                gaussianFilterName = "ddc8_1_0";
                break;
            default:
                ELOG << "No configured " << ddcTypeName() <<
                        " Gaussian filter for a " << iPulsewidth_ns <<
                        " ns pulse";
                return(-1);
                break;
            }
            break;
        }
        case p7142sd3c::DDC4DECIMATE: {

            // Filter width, in integer nanoseconds
            int iFilterwidth_ns = iPulsewidth_ns / _sd3c.codeLength();

            // Find the gaussian filter coefficient set corresponding to this filter width
            switch (iFilterwidth_ns) {
			case 500:
				// 0.5 uS (75m gate) (24 counts)
	            gaussianFilterName = "ddc4_0_500";
				break;
			case 666:
				// 0.667 uS (100m gate) (32 counts)
	            gaussianFilterName = "ddc4_0_667";
				break;
			case 1000:
				// 1.0 uS (150m gate) (48 counts)
				gaussianFilterName = "ddc4_1_000";
				break;
			case 1333:
				// 1.333 uS (200m gate) (64 counts)
				gaussianFilterName = "ddc4_1_333";
				break;
			case 2666:
				// 2.667 uS (400m gate) (128 counts)
				gaussianFilterName = "ddc4_2_667";
				break;
			default:
	            ELOG << "No configured " << ddcTypeName() <<
	                    " Gaussian filter for a " << iFilterwidth_ns <<
	                    " ns filter";
	            return(-1);
				break;
            }
            if (_sd3c.codeLength() > 1) {
            	// If we are using pulse coding, choose the gaussian filter variant for that.
            	gaussianFilterName += "_pulsecode";
            }

            DLOG << "DDC4 filter width: " << iFilterwidth_ns << " ns";
            break;
        }
        case p7142sd3c::DDC10DECIMATE: {
            switch (iPulsewidth_ns) {
            case 480:
            case 500:
            case 520:
                // 0.5 us pulse
//                gaussianFilterName = "ddc10_0_500";
                gaussianFilterName = "ddc10_0_500_flat";
                break;
            case 1000:
                // 1.0 us pulse
                gaussianFilterName = "ddc10_1_000";
                break;
            default:
                ELOG << "No configured " << ddcTypeName() <<
                        " Gaussian filter for a " << iPulsewidth_ns <<
                        " ns pulse";
                return(-1);
                break;
            }
            break;
        }
        default: {
            ELOG << "DDC type " << ddcTypeName() << 
                " not handled in " << __FUNCTION__;
            return -1;
            break;
        }
        }

        if (gaussianFilterName.size() == 0) {
            ELOG << "No configured " << ddcTypeName() <<
                    " Gaussian filter for a " << iPulsewidth_ns << " ns pulse";
            return(-1);
        }
        if (builtins.find(gaussianFilterName) == builtins.end()) {
            ELOG << "No BuiltinGaussian entry for " << gaussianFilterName <<
                    " (" << iPulsewidth_ns << " ns pulsewidth)!";
            return -1;
        }
        gaussian = FilterSpec(builtins[gaussianFilterName]);
        DLOG << "Using gaussian filter coefficient set "
             << gaussianFilterName;
    }

    // get the kaiser filter coefficients
    std::string kaiserFilterName;
    FilterSpec kaiser;
    if (_kaiserFile.size() != 0) {
        FilterSpec k(_kaiserFile);
        if (!k.ok()) {
            ELOG << "Incorrect or unaccessible filter definition: "
                    << _kaiserFile;
            return -1;
        } else {
            kaiser = k;
        }
    } else {
        BuiltinKaiser builtins;
        std::string kaiserFilterName;
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC8DECIMATE: {
            kaiserFilterName = "ddc8_5_0";
            break;
        }
        case p7142sd3c::DDC4DECIMATE: {
            kaiserFilterName = "ddc4_4_0";
            break;
        }
        case p7142sd3c::DDC10DECIMATE: {
            kaiserFilterName = "ddc10_5_0";
            break;
        }
        default: {
            ELOG << "DDC type " << ddcTypeName() << 
                " not handled in " << __FUNCTION__;
            return -1;
            break;
        }
        }
        if (builtins.find(kaiserFilterName) == builtins.end()) {
            ELOG << "No entry for " << kaiserFilterName
                    << " in the list of builtin Kaiser filters!";
            return -1;
        }
        kaiser = FilterSpec(builtins[kaiserFilterName]);
        DLOG << "Using kaiser filter coefficient set " << kaiserFilterName;
    }

    // load the filter coefficients

    if (!loadFilters(gaussian, kaiser)) {
      ELOG << "Unable to load filters";
      if (! usingInternalClock()) {
        ELOG << "Is the external clock source connected to the Pentek?";
        ELOG << "Is the clock signal strength at least +3 dBm?";
      }
      return -1;
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cDn::fifoConfig() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // The fifos need to be configured for
    // the given channel that we are using.

    // find the fifo configuration register
    unsigned int readBack;
    int ppOffset = ADC_FIFO_CTRL_1;
    switch (_chanId) {
    case 0:
        ppOffset = ADC_FIFO_CTRL_1;
        break;
    case 1:
        ppOffset = ADC_FIFO_CTRL_2;
        break;
    case 2:
        ppOffset = ADC_FIFO_CTRL_3;
        break;
    case 3:
        ppOffset = ADC_FIFO_CTRL_4;
        break;
    }

    P7142_REG_READ(_sd3c._BAR2Base + ppOffset, readBack);

    // And configure ADC FIFO Control for this channel
    P7142_REG_WRITE(_sd3c._BAR2Base + ppOffset, readBack & 0x000034BF);

}

//////////////////////////////////////////////////////////////////////////////////
//
// ******    Buffer management and data handling in the following section    *****
//
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
int
p7142sd3cDn::_simulatedRead(char* buf, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Generate simulated data
    makeSimData(n);

    // copy to user buffer
    for (int i = 0; i < n; i++) {
        buf[i] = _simFifo[0];
        _simFifo.pop_front();
    }

    return n;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::getBeam(int64_t & nPulsesSinceStart, float & angle1,
        float & angle2, bool & xmitPolHorizontal) {
    // This method only works for pulse-tagged data
    if (_sd3c._operatingMode() != p7142sd3c::MODE_PULSETAG) {
        ELOG << __PRETTY_FUNCTION__ << " only works for MODE_PULSETAG";
        abort();
    }
    return(ptBeamDecoded(nPulsesSinceStart, angle1, angle2, xmitPolHorizontal));
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::getBeam(int64_t & nPulsesSinceStart,
                     void *metaDataBuf /* = NULL */,
                     int bufLen /* = 0 */) {
    // This method only works for pulse-tagged data
    if (_sd3c._operatingMode() != p7142sd3c::MODE_PULSETAG) {
        ELOG << __PRETTY_FUNCTION__ << " only works for MODE_PULSETAG";
        abort();
    }
    return(ptBeamWithMeta(nPulsesSinceStart, metaDataBuf, bufLen));
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::getBeamOnly(int64_t & nPulsesSinceStart) {

    // perform the simulation wait if necessary
    //if (isSimulating()) {
    //    simWait();
    //}

    switch (_sd3c._operatingMode()) {
        case p7142sd3c::MODE_FREERUN:
            nPulsesSinceStart = 0;
            return frBeam();
        case p7142sd3c::MODE_PULSETAG:
            return ptBeamDecoded(nPulsesSinceStart);
        case p7142sd3c::MODE_CI:
            return ciBeamDecoded(nPulsesSinceStart, false);
        case p7142sd3c::MODE_CI_RIM:
            return ciBeamDecoded(nPulsesSinceStart, true);
        default:
            ELOG << __PRETTY_FUNCTION__ << ": unhandled mode " << 
                _sd3c._operatingMode();
            abort();
            break;
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3cDn::beamLength() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return _beamLength;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ptBeamDecoded(int64_t & nPulsesSinceStart) {
    float angle1;
    float angle2;
    bool xmitPolHorizontal;
    return(ptBeamDecoded(nPulsesSinceStart, angle1, angle2, xmitPolHorizontal));
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ptBeamDecoded(int64_t & nPulsesSinceStart, float & angle1,
        float & angle2, bool & xmitPolHorizontal) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam
    char pulseTag[4];
    char pulseMetadata[ptMetadataLen()];
    char* buf = ptBeam(pulseTag, pulseMetadata);

    // unpack the channel number and pulse sequence number.
    // Unpack the 4-byte channel id/pulse number
    unsigned int chan, pulseNum;
    unpackPtChannelAndPulse(pulseTag, chan, pulseNum);
    if (int(chan) != _chanId) {
        std::ostringstream msgStream;
        msgStream << std::setfill('0');
        msgStream << "On channel " << chan << ", got BAD pulse tag 0x" << 
            std::hex << std::setw(8) << pulseTag << " after pulse tag 0x" <<
            std::setw(8) << (uint32_t(_chanId) << 30 | _lastPulse) << 
            std::dec << ". Pulse number will just be incremented.\n";
        ELOG << msgStream.str();

        // Just hijack the next pulse number, since we've got garbage for
        // the pulse anyway...
        pulseNum = _lastPulse + 1;
    }

    // Unpack the metadata
    if (ptMetadataLen()) {
        unpackPtMetadata(pulseMetadata, angle1, angle2, xmitPolHorizontal);
    }

    // Initialize _lastPulse if this is the first pulse we've seen
    if (_firstBeam) {
        _lastPulse = pulseNum - 1;
        _firstBeam = false;
    }

    // Handle pulse number rollover gracefully
    if (_lastPulse == MAX_PT_PULSE_NUM) {
        DLOG << "Pulse number rollover on channel " << chanId();
        _lastPulse = -1;
    }

    // How many pulses since the last one we saw?
    int delta = pulseNum - _lastPulse;
    if (delta < (-MAX_PT_PULSE_NUM / 2)) {
        delta += MAX_PT_PULSE_NUM + 1;
    }

    if (delta == 0) {
        ELOG << "Channel " << _chanId << ": got repeat of pulse " <<
                pulseNum << "!";
        abort();
    } else if (delta != 1) {
        if (delta < 0) {
            ELOG << "Channel " << _chanId << " went BACKWARD " <<
                -delta << " pulses: " << _lastPulse << "->" << pulseNum;
        } else {
            ELOG << "Channel " << _chanId << " dropped " <<
                delta - 1 << " pulses: " << _lastPulse << "->" << pulseNum;
        }
    }

    _nPulsesSinceStart += delta;
    nPulsesSinceStart = _nPulsesSinceStart;

    _droppedPulses += (delta - 1);
    _lastPulse = pulseNum;

    return buf;
}

//////////////////////////////////////////////////////////////////////////////////
//
// Get a beam, passing in a buffer for the metadata from the start of the beam
// This will contain angles, scan flags etc that are interpreted by the
// calling routine.
// metaDataBuf must be allocated by the caller, to have the length bufLen,

char*
  p7142sd3cDn::ptBeamWithMeta(int64_t & nPulsesSinceStart,
                              void *metaDataBuf /*= NULL */,
                              int bufLen /* = 0 */) {

  boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam
    char pulseTag[4];
    char pulseMetadata[ptMetadataLen()];
    char* buf = ptBeam(pulseTag, pulseMetadata);

    // unpack the channel number and pulse sequence number.
    // Unpack the 4-byte channel id/pulse number
    unsigned int chan, pulseNum;
    unpackPtChannelAndPulse(pulseTag, chan, pulseNum);
    if (int(chan) != _chanId) {
        std::ostringstream msgStream;
        msgStream << std::setfill('0');
        msgStream << "On channel " << chan << ", got BAD pulse tag 0x" << 
            std::hex << std::setw(8) << pulseTag << " after pulse tag 0x" <<
            std::setw(8) << (uint32_t(_chanId) << 30 | _lastPulse) << 
            std::dec << ". Pulse number will just be incremented.\n";
        ELOG << msgStream.str();

        // Just hijack the next pulse number, since we've got garbage for
        // the pulse anyway...
        pulseNum = _lastPulse + 1;
    }

    // Copy the metadata into the client buffer
    if (ptMetadataLen()) {
      int ncopy = ptMetadataLen();
      if (ncopy > bufLen) {
        ncopy = bufLen;
      }
      memcpy(metaDataBuf, pulseMetadata, ncopy);
    }

    // Initialize _lastPulse if this is the first pulse we've seen
    if (_firstBeam) {
        _lastPulse = pulseNum - 1;
        _firstBeam = false;
    }

    // Handle pulse number rollover gracefully
    if (_lastPulse == MAX_PT_PULSE_NUM) {
        DLOG << "Pulse number rollover on channel " << chanId();
        _lastPulse = -1;
    }

    // How many pulses since the last one we saw?
    int delta = pulseNum - _lastPulse;
    if (delta < (-MAX_PT_PULSE_NUM / 2)) {
        delta += MAX_PT_PULSE_NUM + 1;
    }

    if (delta == 0) {
        ELOG << "Channel " << _chanId << ": got repeat of pulse " <<
                pulseNum << "!";
        abort();
    } else if (delta != 1) {
        ELOG << _lastPulse << "->" << pulseNum << ": ";
        if (delta < 0) {
            ELOG << "Channel " << _chanId << " went BACKWARD " <<
                -delta << " pulses";
        } else {
            ELOG << "Channel " << _chanId << " dropped " <<
                delta - 1 << " pulses";
        }
    }

    _nPulsesSinceStart += delta;
    nPulsesSinceStart = _nPulsesSinceStart;

    _droppedPulses += (delta - 1);
    _lastPulse = pulseNum;

    return buf;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ptBeam(char* pulseTag, char* metadata) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

	// How many sync errors at start?
    unsigned long startSyncErrors = _syncErrors;

    // Number of bytes for a complete beam: 4 byte pulse tag +
    // DDC-specific extra metadata + data size (i.e., _beamLength) +
    // 4-byte sync word.
    const uint32_t BytesPerBeam = 4 + ptMetadataLen() + _beamLength + 4;

    // Temporary buffer to hold the 4-byte pulse tag, _beamLength bytes of data,
    // and the trailing sync word.
    char tmpBuf[BytesPerBeam];

    // Keep track of how many useful bytes are currently in tmpBuf.
    int nInTmp = 0;
    
    int r;
    while(1) {
        if (_firstRawBeam) {
            // skip over the first 4 bytes, assuming that
            // they are a good sync word.
            r = read(tmpBuf, 4);
            assert(r == 4);
            _firstRawBeam = false;
        }

        // Read the 4-byte pulse tag, extra metadata, IQ beam data, and 4-byte
        // sync word into tmpBuf (_beamLength + ptMetadataLen() +
        // 8 bytes). Generally, we will read the full length here, but we may
        // read fewer if we still have data in tmpBuf after hunting for a sync
        // word (see below).
        int nToRead = BytesPerBeam - nInTmp;
        r = read(tmpBuf + nInTmp, nToRead);
        assert(r == nToRead);
        
        nInTmp = BytesPerBeam;

        // Copy out the pulse tag from the beginning, the data bytes from the
        // middle, and (what should be) the sync word from the end.
        memcpy(pulseTag, tmpBuf, 4);
        
        // Copy out the metadata
        memcpy(metadata, tmpBuf + 4, ptMetadataLen());

        // Copy the IQ data into _buf
        memcpy(_buf, tmpBuf + 4 + ptMetadataLen(), _beamLength);

        uint32_t word;
        memcpy(&word, tmpBuf + BytesPerBeam - 4, 4);

        // If we are indeed in sync, return the good pulse data now
        if (word == SD3C_SYNCWORD) {
            break;
        }
            
        // No sync? Hunt word-by-word until we find a sync word, then go 
        // back to the top. Start looking in the stuff we already read, then
        // read beyond that if necessary.
        _syncErrors++;

        // Keep information about the words we're skipping to find the next sync
        std::ostringstream syncHuntMsg;

        uint32_t nHuntWords = 0;
        uint32_t consecutiveData = 0;
        
        syncHuntMsg << "Sync hunt words on card: " << _p7142.getCardIndex()
                    << ", chan: " << _chanId << " : ";
        syncHuntMsg << std::setfill('0');

        while (true) {
            // If we still have data that we read above, search through it
            // looking for the sync word. If we run out of previously read
            // data, then read in one new word at a time.
            if (nHuntWords < (BytesPerBeam / 4)) {
                memcpy(&word, tmpBuf + nHuntWords * 4, 4);
            } else {
                r = read(reinterpret_cast<char*>(&word), 4);
                assert(r == 4);
            }
            nHuntWords++;

            // Break out when we've found a sync word
            if (word == SD3C_SYNCWORD) {
                // Keep any remaining bytes after the sync word in tmpBuf,
                // moving them to the beginning of tmpBuf.
                if (4 * nHuntWords < sizeof(tmpBuf)) {
                    memmove(tmpBuf, tmpBuf + 4 * nHuntWords, 
                            BytesPerBeam - 4 * nHuntWords);
                    nInTmp -= 4 * nHuntWords;
                } else {
                    nInTmp = 0;
                }
                // Break out, since we found a sync word
                break;
            }

            // If the bad word can be broken into two 16-bit numbers with
            // absolute value < 32, then consider it an I and Q data word.
            // IMPORTANT NOTE: this assumes we have only noise on the channel, 
            // hence low I and Q values.
            //
            // Otherwise, print the word as being interesting (likely a pulse 
            // tag).
            int16_t shortp[2];
            memcpy(shortp, &word, sizeof(word));
            // int16_t * shortp = reinterpret_cast<int16_t *>(&word);
            if ((shortp[0] > -32 && shortp[0] < 32) && 
                (shortp[1] > -32 && shortp[1] < 32)) {
                consecutiveData++;
            } else {
                // Report consecutive data words before this word
                if (consecutiveData) {
                    syncHuntMsg << "<DATA>x" << consecutiveData;
                    consecutiveData = 0;
                }
                // Then the current interesting word
                syncHuntMsg << "<" << std::setw(8) << std::hex << word << ">" <<
                    std::dec;
            }
        }
        
        if (consecutiveData) {
            syncHuntMsg << "<DATA>x" << consecutiveData;
        }
        syncHuntMsg << "<SYNC>";
        ELOG << syncHuntMsg.str();
    }

    if (_syncErrors != startSyncErrors) {
        uint32_t * wordp = reinterpret_cast<uint32_t *>(pulseTag);
        ELOG << std::setfill('0');
        ELOG << "XX Got " << _syncErrors - startSyncErrors
             << " sync errors, cardIndex: " << _p7142.getCardIndex()
             << ", channel " << chanId();
        ELOG << " finding pulse w/tag 0x"
             << std::setw(8) << std::hex << *wordp
             << " after tag 0x" << std::setw(8)
             << (uint32_t(_chanId) << 30 | _lastPulse) << std::dec;
    }
    return _buf;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ciBeamDecoded(int64_t& nPulsesSinceStart, bool rim) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam
    unsigned int pulseNum;
    char* buf;
    if (!rim) { 
    	buf = ciBeam(pulseNum);
    } else {
    	buf = ciBeamRim(pulseNum);
    }

    // Initialize _lastPulse if this is the first pulse we've seen
    if (_firstBeam) {
        _lastPulse = pulseNum - 1;
        _firstBeam = false;
    }

    // How many pulses since the last one we saw?
    int delta = pulseNum - _lastPulse;
    if (delta < (-MAX_CI_PULSE_NUM / 2)) {
        // if the new pulse number is zero, assume that it
        // was a legitimate wrap. Unfortunately this won't catch
        // errors where the zero pulse is skipped, or a pulse comes in
        // that erroneously has zero for a pulse tag. Perhaps there
        // is a better algorithm for this.
        if (pulseNum == 0)
            DLOG << "Pulse number rollover";

        delta += MAX_CI_PULSE_NUM + 1;

    }

    if (delta == 0) {
        ELOG << "Channel " << _chanId << ": got repeat of pulse " <<
                pulseNum << "!";
        abort();
    } else if (delta != 1) {
        //ELOG << _lastPulse << "->" << pulseNum << ": ";
        if (delta < 0) {
            //ELOG << "Channel " << _chanId << " went BACKWARD " <<
            //    -delta << " pulses";
        } else {
            //ELOG << "Channel " << _chanId << " dropped " <<
            //    delta - 1 << " pulses";
        }
    }

    _nPulsesSinceStart += delta;
    nPulsesSinceStart = _nPulsesSinceStart;

    _droppedPulses += (delta - 1);
    _lastPulse = pulseNum;

    return buf;
}
//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ciBeam(unsigned int& pulseNum) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r;

    while(1) {
        if (_firstRawBeam) {
            // skip over the first 16 bytes, assuming that
            // they are a good tag word.
            r = read(_buf, 16);
            assert(r == 16);
            _firstRawBeam = false;
        }

        // read one beam into buf
        r = read(_buf, _beamLength);
        assert(r == _beamLength);

        // read the next tag word
        char tagbuf[16];
        r = read(tagbuf, 16);
        assert(r == 16);

        if (ciCheckTag(tagbuf, pulseNum)) {
            return _buf;
        }
        _syncErrors++;
        
        // scan 4 bytes at a time for a correct tag
        while(1) {
            memmove(tagbuf, tagbuf+4,12);
            r = read(tagbuf+12, 4);
            assert(r == 4);
            // check for synchronization
            if (ciCheckTag(tagbuf, pulseNum)) {
                break;
            }
        }
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ciBeamRim(unsigned int& pulseNum) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (0) {
		std::cout << "beam length is " << beamLength()
				<< " (" << beamLength()/4 << " I/Q words)" << std::endl;
		uint32_t bigbuf[2000];
		read((char*)bigbuf, 4*2000);
		for (int i = 0; i < 2000; i++) {
			if (!(i %16)) {
				std::cout << std::endl  << std::setw(6) << std::setfill(' ') << std::dec << i << " ";
			}
			std::cout << std::setw(8) << std::setfill('0') << std::hex << bigbuf[i] << " ";
		}
		std::cout << std::dec;
    }

    int r;
    while(1) {
        if (_firstRawBeam) {
            // skip over the first 64 bytes, assuming that
            // they are a good tag word.
            r = read(_buf, 64);
            assert(r == 64);
            _firstRawBeam = false;
        }

        // read one beam into buf
        r = read(_buf, _beamLength);
        assert(r == _beamLength);

        // read the next tag word
        char tagbuf[64];
        r = read(tagbuf, 64);
        assert(r == 64);

        if (ciCheckTagRim(tagbuf, pulseNum)) {
            return _buf;
        }
        _syncErrors++;

        // scan 4 bytes at a time for a correct tag
        while(1) {
            memmove(tagbuf, tagbuf+4,12);
            r = read(tagbuf+12, 4);
            assert(r == 4);
            // check for synchronization
            if (ciCheckTagRim(tagbuf, pulseNum)) {
                break;
            }
        }
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::frBeam() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r = read(_buf, _beamLength);
    assert(r == _beamLength);
    return _buf;
}

//////////////////////////////////////////////////////////////////////////////////
bool
p7142sd3cDn::ciCheckTag(char* p, unsigned int& pulseNum) {

// The tag and data order:
//  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) IQpairs,odd pulse
//
// The CI tag:
//  bits 31:28  Format number   0-15(4 bits)
//  bits 27:26  Channel number  0-3 (2 bits)
//  bits    25  0=even, 1=odd   0-1 (1 bit)
//  bit     24  0=I, 1=Q        0-1 (1 bit)
//  bits 23:00  Sequence number     (24 bits)

    int format[4];
    int chan[4];
    bool Odd[4];
    bool Q[4];
    uint32_t seq[4];
    for (int i = 0; i < 4; i++) {
        uint32_t* tag = (uint32_t*)p;
        ciDecodeTag(tag[i], format[i], chan[i], Odd[i], Q[i], seq[i]);
    }

    pulseNum = seq[0];

    // time to see if we received expected values
    bool retval = true;

    retval     = retval && (format[0] ==      1);

    for (int i = 1; i < 4; i++) {
        retval = retval && (format[i] ==      1);
        retval = retval && (seq[i]    == seq[0]);
        retval = retval && (chan[i]   == chan[0]);
    }
    retval = retval && !Odd[0] && !Odd[1] && Odd[2] && Odd[3];
    retval = retval &&   !Q[0] &&    Q[1] &&  !Q[2] &&   Q[3];

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
bool
p7142sd3cDn::ciCheckTagRim(char* p, unsigned int& pulseNum) {

	//  In range imaging mode, the tags are repeated four times (once per frequency)
	//
	// The tag and data order:
	// (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
	// (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
	// (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
	// (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
	// (IQpairs,even_pulse) IQpairs,odd pulse
	//
	// The CI tag:
	//  bits 31:28  Format number   0-15(4 bits)
	//  bits 27:26  Channel number  0-3 (2 bits)
	//  bits    25  0=even, 1=odd   0-1 (1 bit)
	//  bit     24  0=I, 1=Q        0-1 (1 bit)
	//  bits 23:00  Sequence number     (24 bits)

    int format[16];
    int chan[16];
    bool Odd[16];
    bool Q[16];
    uint32_t seq[16];
    for (int f = 0; f < 4; f++) {
		for (int i = 0; i < 4; i++) {
			uint32_t* tag = (uint32_t*)p;
			int index = 4*f + i;
			ciDecodeTag(tag[index], format[index], chan[index], Odd[index], Q[index], seq[index]);
		}
    }

    pulseNum = seq[0];

    // time to see if we received expected values
    bool retval = true;

	retval     = retval && (format[0] ==      2);

	for (int f = 0; f < 4; f++) {
		for (int i = 1; i < 4; i++) {
			retval = retval && (format[i+4*f] ==      2);
			retval = retval && (seq[i+4*f]    == seq[0]);
			retval = retval && (chan[i+4*f]   == chan[0]);
		}
		retval = retval && !Odd[0+4*f] && !Odd[1+4*f] && Odd[2+4*f] && Odd[3+4*f];
		retval = retval &&   !Q[0+4*f] &&    Q[1+4*f] &&  !Q[2+4*f] &&   Q[3+4*f];
	}

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t
p7142sd3cDn::ciMakeTag(int format, int chan, bool odd, bool Q, uint32_t seq) {
    /// The CI tag:
    ///  bits 31:28  Format number   0-15(4 bits)
    ///  bits 27:26  Channel number  0-3 (2 bits)
    ///  bits    25  0=even, 1=odd   0-1 (1 bit)
    ///  bit     24  0=I, 1=Q        0-1 (1 bit)
    ///  bits 23:00  Sequence number     (24 bits)

    int Odd = odd? 1:0;
    int IQ   =  Q? 1:0;
    uint32_t tag =
    		(( format << 4 | chan << 2 | Odd << 1 | IQ) << 24) | (seq & 0xffffff);

    return tag;
    
    std::ostringstream stream;
    stream << "format: " << format << " chan:" << chan 
           << " odd:" << odd << " Q:" << Q;
    stream.width(8);
    stream.fill('0');
    stream << std::hex << tag;
    DLOG << stream.str();

    return tag;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::ciDecodeTag(uint32_t tag, int& format, int& chan, bool& odd, bool& Q, uint32_t& seq) {
    /// The CI tag, in little endian format as described in VHDL:
    ///  bits 31:28  Format number   0-15(4 bits)
    ///  bits 27:26  Channel number  0-3 (2 bits)
    ///  bits    25  0=even, 1=odd   0-1 (1 bit)
    ///  bit     24  0=I, 1=Q        0-1 (1 bit)
    ///  bits 23:00  Sequence number     (24 bits)

    format =        (tag >> 28) & 0xf;
    chan   =        (tag >> 26) & 0x3;
    odd    = (bool) ((tag >> 25) & 0x1);
    Q      = (bool) ((tag >> 24) & 0x1);
    seq    =        (tag & 0xffffff);

    return;

    std::ostringstream stream;
    stream << "decoded format: " << format << " chan:"
           << chan << " odd:" << odd << " Q:" << Q
           << " seq:" << seq;
    stream.width(8);
    stream.fill('0');
    stream << " decoded tag:" << std::hex << tag << std::dec;
    //DLOG << stream.str();
    std::cout << stream.str() << std::endl;

    return;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::initBuffer() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // note that _beamLength is only the length of the
    // IQ data (in bytes).

    switch(_sd3c._operatingMode()) {
    case p7142sd3c::MODE_FREERUN:
        // free run mode has:
        //   16 bit I and Q pairs for each gate
        _beamLength = _gates * 2 * 2;
      break;
    case p7142sd3c::MODE_PULSETAG:
        // pulse tag mode has:
        //    16 bit I and Q pairs for each gate
        _beamLength = _gates * 2 * 2;
        break;
    case p7142sd3c::MODE_CI:
        // coherent integration mode has:
        //   even 32 bit I and Q pairs followed by
        //   odd  32 bit I and Q pairs,
        // for each gate.
        _beamLength = _gates * 2 * 2 * 4;
        break;
    case p7142sd3c::MODE_CI_RIM:
        // RIM coherent integration mode has:
        //   even 32 bit I and Q pairs followed by
        //   odd  32 bit I and Q pairs,
        // for each gate,
    	// for 4 frequencies.
        _beamLength = 4*(_gates * 2 * 2 * 4);
        break;
    default:
        ELOG << __PRETTY_FUNCTION__ << ": unknown SD3C mode: " << 
            _sd3c._operatingMode();
        abort();
        break;
    }

    // allocate the buffer to hold one (or two for CI) beams of IQ data
    _buf = new char[_beamLength];

}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::makeSimData(int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r;

    while(_simFifo.size() < (unsigned int)n) {
        switch(_sd3c._operatingMode()) {
        case p7142sd3c::MODE_FREERUN: {
            // ************* free run mode ***************
            for (int i = 0; i < _beamLength/4; i++) {
                uint32_t iq;
                char* p = (char*)&iq;
                r = p7142Dn::_simulatedRead(p, 4);
                assert(r == 4);
                for (int j = 0; j < 4; j++) {
                    _simFifo.push_back(p[j]);
                }
            }
            break;
        }
        case p7142sd3c::MODE_PULSETAG: {
            // ********** pulse tag mode **************
            // Add sync word
            uint32_t syncword = SD3C_SYNCWORD;
            for (int i = 0; i < 4; i++) {
                _simFifo.push_back(((char*)&syncword)[i]);
            }
            // Add the pulse tag for this sample:
            //       bits 31:30  Channel number         0-3 (2 bits)
            //       bits 29:00  Pulse sequence number  0-1073741823 (30 bits)
            // This is packed as a little-endian order 4-byte word;
            uint32_t simPulseNum = _sd3c.nextSimPulseNum(_chanId);
            uint32_t tag = (_chanId << 30) | (simPulseNum & 0x3fffffff);
            char* p = (char*)&tag;
            for (int i = 0; i < 4; i++) {
                _simFifo.push_back(p[i]);
            }
            // Add all-zero metadata
            for (int i = 0; i < ptMetadataLen(); i++) {
                _simFifo.push_back(0);
            }
            // Add IQ data. Occasionally drop some data
            bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
            doBadSync = false;
            int nPairs = _beamLength/4;
            if (doBadSync) {
                nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
            }
            for (int i = 0; i < nPairs; i++) {
                uint32_t iq;
                char* p = (char*)&iq;
                r = p7142Dn::_simulatedRead(p, 4);
                assert(r == 4);
                for (int j = 0; j < 4; j++) {
                    _simFifo.push_back(p[j]);
                }
            }
            break;
        }
        case p7142sd3c::MODE_CI: {
            /// Add the coherent integration tag for this sample:

            ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) ((IQpairs,odd_pulse))
            ///
            ///  bits 31:28  Format number   0-15(4 bits)
            ///  bits 27:26  Channel number  0-3 (2 bits)
            ///  bits    25  0=even, 1=odd   0-1 (1 bit)
            ///  bit     24  0=I, 1=Q        0-1 (1 bit)
            ///  bits 23:00  Sequence number     (24 bits)

            uint32_t simPulseNum = _sd3c.nextSimPulseNum(_chanId);
            for (int j = 0; j < 4; j++) {
                //uint32_t tag = ciMakeTag(1, _chanId, (j>>1)&1, j&1, _simPulseNum);
                uint32_t tag = ciMakeTag(1, _chanId, (j>>1)&1, j&1, simPulseNum);
                char* p = (char*)&tag;
                for (int i = 0; i < 4; i++) {
                    _simFifo.push_back(p[i]);
                }
            }

            // Add IQ data. Occasionally drop some data
            bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
            // Disable corrupted sync data for now.
            doBadSync = false;
            // I and Q values from the CI are 4 byte values,
            // so it will take 8 bytes for an I/Q pair.
            int nPairs = _beamLength/8;
            if (doBadSync) {
                nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
            }
            for (int i = 0; i < nPairs; i++) {
                char iq[8];
                r = p7142Dn::_simulatedRead(iq, 8);
                assert(r == 8);
                for (int j = 0; j < 8; j++) {
                    _simFifo.push_back(iq[j]);
                }
            }

            break;
        }
        case p7142sd3c::MODE_CI_RIM: {
            /// Add the coherent integration (RIM) tag for this sample:

            ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
            ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
            ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
            ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
            ///
            ///  bits 31:28  Format number   0-15(4 bits) (==2)
            ///  bits 27:26  Channel number  0-3 (2 bits)
            ///  bits    25  0=even, 1=odd   0-1 (1 bit)
            ///  bit     24  0=I, 1=Q        0-1 (1 bit)
            ///  bits 23:00  Sequence number     (24 bits)

            uint32_t simPulseNum = _sd3c.nextSimPulseNum(_chanId);
            for (int freq = 0; freq < 4; freq++) {
            	for (int j = 0; j < 4; j++) {
					//uint32_t tag = ciMakeTag(2, _chanId, (j>>1)&1, j&1, _simPulseNum);
					uint32_t tag = ciMakeTag(2, _chanId, (j>>1)&1, j&1, simPulseNum);
					char* p = (char*)&tag;
					for (int i = 0; i < 4; i++) {
						_simFifo.push_back(p[i]);
					}
            	}
            }

            // Add IQ data. Occasionally drop some data
            bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
            // Disable corrupted sync data for now.
            doBadSync = false;
            // I and Q values from the CI are 4 byte values,
            // so it will take 8 bytes for an I/Q pair.
            int nPairs = _beamLength/8;
            if (doBadSync) {
                nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
            }
            for (int i = 0; i < nPairs; i++) {
                char iq[8];
                r = p7142Dn::_simulatedRead(iq, 8);
                assert(r == 8);
                for (int j = 0; j < 8; j++) {
                    _simFifo.push_back(iq[j]);
                }
            }

            break;
        }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::unpackPtChannelAndPulse(const char* buf, unsigned int & chan,
        unsigned int & pulseNum) {
    // Channel number is the upper two bits of the channel/pulse num word, which
    // is stored in little-endian byte order
    const unsigned char *ubuf = (const unsigned char*)buf;
    chan = (ubuf[3] >> 6) & 0x3;

    // Pulse number is the lower 30 bits of the channel/pulse num word, which is
    // stored in little-endian byte order
    pulseNum = (ubuf[3] & 0x3f) << 24 | ubuf[2] << 16 | ubuf[1] << 8 | ubuf[0];
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::unpackPtMetadata(const char* buf, float & angle1,
        float & angle2, bool & xmitPolHorizontal) {
    // The angles are packed in the first two 32-bit words of the metadata.
    const uint32_t * ui32vals = reinterpret_cast<const uint32_t *>(buf);
    // The first 32-bit word is the rotation/azimuth angle
    angle1 = (360. / 400000) * ui32vals[0];
    // The second 32-bit word is the tilt/elevation angle
    angle2 = (360. / 480000) * ui32vals[1];
    // Move angle2 into range [-180,180]
    if (angle2 > 180.0) {
        angle2 -= 360.0;
    }
    // Polarization bit is LSB of word 2: 0 if horizontal polarization,
    // 1 if vertical
    xmitPolHorizontal = (ui32vals[2] & 0x1) == 0;

}

//////////////////////////////////////////////////////////////////////////////////
unsigned long
p7142sd3cDn::droppedPulses() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    unsigned long retval = _droppedPulses;
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
unsigned long
p7142sd3cDn::syncErrors() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    unsigned long retval = _syncErrors;
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::dumpSimFifo(std::string label, int n) {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    std::ostringstream out;
    out << label <<  " _simFifo length: " << _simFifo.size() << std::endl;
    out << std::hex;
    for (unsigned int i = 0; i < (unsigned int)n && i < _simFifo.size(); i++) {
        out << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)_simFifo[i] << " ";
        if (!((i+1) % 40)) {
            out << std::endl;
        }
    }
    out << std::dec;
    DLOG << out.str();
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3cDn::ptMetadataLen() const {
    // How much space is added per pulse for metadata?
    switch (_sd3c._ddcType) {
    case p7142sd3c::DDC8DECIMATE:
        // DDC8 has 6 extra words (24 bytes) of metadata
        return(24);
        break;
    case p7142sd3c::DDC10DECIMATE:
        // DDC10 (after rev. 535) has 6 extra words (24 bytes) of metadata
        if (_sd3c.sd3cRev() <= 535) {
            return(0);
        } else {
            return(24);
        }
        break;
    case p7142sd3c::DDC6DECIMATE:
        // DDC6 has 6 extra words (24 bytes) of metadata
        return(24);
    default:
        return(0);
    }
}

} // end namespace Pentek
