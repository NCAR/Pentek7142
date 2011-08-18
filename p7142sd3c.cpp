/*
 * p7142sd3c.cpp
 *
 *  Created on: Jan 26, 2009
 *      Author: martinc
 */

#include "p7142sd3c.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>

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


////////////////////////////////////////////////////////////////////////////////////////
p7142sd3c::p7142sd3c(std::string devName, bool simulate, double tx_delay, 
    double tx_pulsewidth, double prt, double prt2, bool staggeredPrt, 
    unsigned int gates, unsigned int nsum, bool freeRun, 
    DDCDECIMATETYPE simulateDDCType, bool externalStartTrigger) : 
        p7142(devName, simulate),
        _staggeredPrt(staggeredPrt),
        _freeRun(freeRun),
        _gates(gates),
        _nsum(nsum),
        _simulateDDCType(simulateDDCType),
        _externalStartTrigger(externalStartTrigger) {
	boost::recursive_mutex::scoped_lock guard(_mutex);

    // sanity check
    if (_nsum < 1) {
        std::cerr << "Selected nsum of " << _nsum << " makes no sense!" <<
                std::endl;
        abort();
    }

    // Get the firmware revison and ddc type from the FPGA.
	if (simulate) {
		_sd3cRev = 1;
		_ddcType = simulateDDCType;
	} else {
		_sd3cRev = sd3cRev();
		_ddcType = ddcType();
	}

    // Set the ADC clock rate based on DDC type
    switch (_ddcType) {
    case DDC10DECIMATE:
        _adc_clock = 100.0e6;
        break;
    case DDC8DECIMATE:
        _adc_clock = 125.0e6;
        break;
    case DDC4DECIMATE:
        _adc_clock = 48.0e6;
        break;
    case BURST:
        _adc_clock = 100.0e6;
        break;
    }

    // Announce the FPGA firmware revision
    std::cout << _devName << " SD3C revision: " << std::dec << _sd3cRev << std::endl;
    if (_sd3cRev == 0) {
        std::cerr << "** WARNING: Revision number is zero. " <<
                "Was the correct firmware loaded?" << std::endl;
    }

    // Determine our operating mode
    _mode = (_nsum > 1) ? MODE_CI : MODE_PULSETAG;
    if (_freeRun) {
        _mode = MODE_FREERUN;
    }

    // stop the timers
    timersStartStop(false);
    
    // Write the gate count and coherent integration registers
    if (! isSimulating()) {
    	uint32_t temp;

    	P7142_REG_WRITE(BAR2Base + RADAR_GATES, gates);
    	P7142_REG_READ (BAR2Base + RADAR_GATES, temp);
    	std::cout << "gate readback is " << temp <<std::endl;
    	P7142_REG_WRITE(BAR2Base + CI_NSUM, nsum);
    	P7142_REG_READ (BAR2Base + CI_NSUM, temp);
    	std::cout << "nsum readback is " << temp << std::endl;
    }

    // Convert prt, prt2, tx_pulsewidth, and tx_delay into our local representation, 
    // which is in units of (ADC clock counts / 2)
    _prtCounts = timeToCounts(prt);
    _prt2Counts = timeToCounts(prt2);
    _prf = 1.0 / prt;   // Hz
    _prf2 = 1.0 / prt2; // Hz

    // Sync pulse timer. Note that the width of this timer must be at least
    // 140 ns to be recognized to be counted by the Acromag PMC730 Multi-IO
    // card pulse counter, and this counter is used by the Ka-band radar!
    setTimer(MASTER_SYNC_TIMER, 0, timeToCounts(140.e-9));
    
    // tx pulse timer
    int txDelayCounts = timeToCounts(tx_delay);
    int pulseWidthCounts = timeToCounts(tx_pulsewidth);
    setTimer(TX_PULSE_TIMER, txDelayCounts, pulseWidthCounts);
    
    std::cout << "downconverter: " << ddcTypeName(_ddcType) << std::endl;
    std::cout << "tx delay:      " << timerDelay(TX_PULSE_TIMER) << " adc_clock/2 counts"  << std::endl;
    std::cout << "tx pulse width:" << timerWidth(TX_PULSE_TIMER) << " adc_clock/2 counts"   << std::endl;
    std::cout << "prt:           " << _prtCounts       << " adc_clock/2 counts"   << std::endl;
    std::cout << "prt2:          " << _prt2Counts      << " adc_clock/2 counts"   << std::endl;
    std::cout << "staggered:     " << ((_staggeredPrt) ? "true" : "false")        << std::endl;
    std::cout << "gates:         " << _gates                                      << std::endl;
    std::cout << "nsum:          " << _nsum                                       << std::endl;
    std::cout << "free run:      " << ((_freeRun) ? "true" : "false")             << std::endl;
    std::cout << "adc clock:     " << _adc_clock       << " Hz"                   << std::endl;
    std::cout << "prf:           " << _prf             << " Hz"                   << std::endl;
    std::cout << "data rate:     " << dataRate()/1.0e3 << " KB/s"                 << std::endl;

    //    std::cout << "rx 0/1 delay:  " << _timerDelay(RX_01_TIMER) << " adc_clock/2 counts"  << std::endl;
    //    std::cout << "rx 0/1 width:  " << _timerWidth(RX_01_TIMER) << " adc_clock/2 counts"   << std::endl;
    //    std::cout << "rx 2/3 delay:  " << _timerDelay(RX_23_TIMER) << " adc_clock/2 counts"  << std::endl;
    //    std::cout << "rx 2/3 width:  " << _timerWidth(RX_23_TIMER) << " adc_clock/2 counts"   << std::endl;
    //    std::cout << "clock source:  " << (usingInternalClock() ? "internal" : "external") << std::endl;
    //    std::cout << "ts length:     " << _tsLength                                   << std::endl;
    //    std::cout << "sim usleep     " << _simPauseMS*1000 << "us"                    <<std::endl;
	//    for (int i = 0; i < 8; i++) {
	//        std::cout << "timer " << i << " delay: " << _timerDelay(i) << " adc_clock/2 counts"  << std::endl;
	//        std::cout << "timer " << i << " width: " << _timerWidth(i) << " adc_clock/2 counts"  << std::endl;
	//    }
    
    // reset the FPGA clock managers. Necessary since some of our
    // new DCMs in the firmware use the CLKFX output, which won't
    // lock at startup.
    resetDCM();

    // set free run mode as appropriate
    loadFreeRun();
}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3c::~p7142sd3c() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn*
p7142sd3c::addDownconverter(int chanId, bool burstSampling, int tsLength,
        double rx_delay, double rx_pulse_width, std::string gaussianFile, 
        std::string kaiserFile, double simPauseMs, int simWavelength,
        bool internalClock) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Create a new p7142sd3cDn downconverter and put it in our list
    p7142sd3cDn* downconverter = new p7142sd3cDn(
    		this,
			chanId,
			burstSampling,
			tsLength,
			rx_delay,
			rx_pulse_width,
			gaussianFile,
			kaiserFile,
			simPauseMs,
			simWavelength,
			internalClock);

    p7142::addDownconverter(downconverter);

    return(downconverter);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142sd3c::setTimer(TimerIndex ndx, int delay, int width, bool verbose, bool invert) {

    _TimerConfig currentVals = _timerConfigs[ndx];

    boost::recursive_mutex::scoped_lock guard(_mutex);

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

    _timerConfigs[ndx] = _TimerConfig(delay, width, invert);
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3c::timeToCounts(double time) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    return(lround(time * _adc_clock / 2));
}

//////////////////////////////////////////////////////////////////////////////////
double
p7142sd3c::countsToTime(int counts) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    return((2 * counts) / _adc_clock);
}

/////////////////////////////////////////////////////////////////////////
void p7142sd3c::timersStartStop(bool start) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate) {
        setXmitStartTime(microsec_clock::universal_time());
        return;
    }

    // Load timer values before starting the timers
    if (start) {
        initTimers();
    }
        
    // Turn on Write Strobes
    P7142_REG_WRITE(BAR2Base + MT_WR, WRITE_ON);

    // configure each timer
    for (int i = 0; i < 8; i++) {
	    // Control Register
    	P7142_REG_WRITE(BAR2Base + MT_ADDR, CONTROL_REG | SD3C_TIMER_BITS[i]);
	
	    // Enable/Disable Timer
        unsigned int value =
        		(start ? TIMER_ON : 0) | (timerInvert(i) ? TIMER_NEG : 0);

        P7142_REG_WRITE(BAR2Base + MT_DATA, value);
    }

    // Get current time
    ptime now(microsec_clock::universal_time());
    //
    // Actually start or stop the timers now
    //
    if (start) {
        if (_externalStartTrigger) {
            // We assume here that the external trigger is a 1 PPS signal, 
            // e.g., from GPS.
            //
            // Sleep until ~0.2 seconds after the top of a second. This gives
            // us a comfortable fraction of a second to set up timer start and 
            // know at precisely which second the timers will start. It also 
            // allows for our system clock to be off by up to 0.2 seconds.
            
            // sleep until the next 0.2 second mark
            int wake_uSec = 200000; // wake at 0.2 seconds after the top of a second
            int usecNow = now.time_of_day().total_microseconds() % 1000000;
            int sleep_uSec = (1000000 + wake_uSec - usecNow) % 1000000;
            // Timers will start at the top of the next second after we wake
            setXmitStartTime(now + microseconds(1000000 + sleep_uSec - wake_uSec));
            // Now sleep
            usleep(sleep_uSec);
            // Set the wait-for-trigger bit so timers start at the next
            // trigger.
            P7142_REG_WRITE(BAR2Base + MT_ADDR, ALL_SD3C_TIMER_BITS | GPS_EN);
        } else {
            // Internal trigger: timers start immediately.
            setXmitStartTime(now);
            P7142_REG_WRITE(BAR2Base + MT_ADDR, ALL_SD3C_TIMER_BITS | ADDR_TRIG);
        }
        
        std::cout << "Timers/radar start time " << _xmitStartTime << std::endl;
    } else {
    	P7142_REG_WRITE(BAR2Base + MT_ADDR, ALL_SD3C_TIMER_BITS);
        std::cout << "Timers stopped at " << now << std::endl;
    }
    
    // Turn off Write Strobes
    P7142_REG_WRITE(BAR2Base + MT_WR, WRITE_OFF);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::startFilters() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // Start the DDC
    P7142_REG_WRITE(BAR2Base + KAISER_ADDR, DDC_START);

    usleep(p7142::P7142_IOCTLSLEEPUS);

    std::cout << "filters enabled on " << _devName << std::endl;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::stopFilters() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    uint32_t temp;
    // stop the filters if they are running.
    P7142_REG_READ (BAR2Base + KAISER_ADDR, temp);
    P7142_REG_WRITE(BAR2Base + KAISER_ADDR, DDC_STOP);
    P7142_REG_READ (BAR2Base + KAISER_ADDR, temp);
}

//////////////////////////////////////////////////////////////////////
unsigned short int p7142sd3c::TTLIn() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return 0;

    uint32_t val;
    P7142_REG_READ(BAR2Base + TTL_IN, val);

    return val;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::TTLOut(unsigned short int data) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return;

    P7142_REG_WRITE(BAR2Base + TTL_OUT1, data);

}

//////////////////////////////////////////////////////////////////////
unsigned int p7142sd3c::sd3cTypeAndRev() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return 1;

    uint32_t retval;

    P7142_REG_READ(BAR2Base + FPGA_REPO_REV, retval);

    return retval;

}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::sd3cRev() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return _simulateDDCType;

    unsigned int ddcTypeAndRev = sd3cTypeAndRev();

    // Up to rev 502, DDC type was a 1-bit value at bit 15.
    // After that it's a 2-bit value in bits 14-15.
    int retval = (ddcTypeAndRev & 0x3fff > 502) ?
        (ddcTypeAndRev & 0x3fff) : (ddcTypeAndRev & 0x7fff);

    return retval;
}

//////////////////////////////////////////////////////////////////////
p7142sd3c::DDCDECIMATETYPE p7142sd3c::ddcType() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return _simulateDDCType;
    
    unsigned int ddcTypeAndRev = sd3cTypeAndRev();
    std::cout << "std type and rev " << std::hex << ddcTypeAndRev << std::dec << std::endl;

    // Up to rev 502, DDC type was a 1-bit value at bit 15.
    // After that it's a 2-bit value in bits 14-15.
    int ddcType = (ddcTypeAndRev & 0x3fff > 502) ?
        (ddcTypeAndRev & 0xC000) >> 14 : (ddcTypeAndRev & 0x8000) >> 15;
    
    DDCDECIMATETYPE retval = DDC4DECIMATE;
    switch (ddcType) {
    case 0:
    	retval = DDC4DECIMATE;
        break;
    case 1:
    	retval = DDC8DECIMATE;
        break;
    case 2:
    	retval = DDC10DECIMATE;
        break;
    case 3:
    	retval = BURST;
        break;     
    }
    
    return retval;
}

//////////////////////////////////////////////////////////////////////
void
p7142sd3c::loadFreeRun() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // get the transceiver control register
    uint32_t tcreg;
    P7142_REG_READ(BAR2Base + TRANS_CNTRL, tcreg);

    // set the free run bit as specified by _freerun
    if (_freeRun) {
        // set free run
    	P7142_REG_WRITE(BAR2Base + TRANS_CNTRL, tcreg | TRANS_FREE_RUN);
    } else {
        // clear free run
    	P7142_REG_WRITE(BAR2Base + TRANS_CNTRL, tcreg & ~TRANS_FREE_RUN);
    }

}

/////////////////////////////////////////////////////////////////////////
bool
p7142sd3c::initTimers() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return true;

    //    This section initializes the timers.

    int periodCount; // Period Count for all Timers
    int PrtScheme; // PRT Scheme for all Timers

    // Calculate the period and PRT Scheme for dual prt or single prt
    // Note: _prtCounts and _prt2Counts are expressed in ADC_Clk/2 MHz Counts!
    //       for DDC4: 24 MHz; for DDC8: 62.5 MHz

    int X, Y;
    float prt_ms, prt2_ms;

    if (_staggeredPrt == true) {
        // dual prt
        prt_ms = countsToTime(_prtCounts) * 1000;
        prt2_ms = countsToTime(_prt2Counts) * 1000;

        periodCount = timeToCounts(prt_ms * (prt2_ms / prt_ms - (int) (prt2_ms
                / prt_ms)) / (int) (prt2_ms / prt_ms) * 0.001);

        X = (int) ((int) (prt2_ms / prt_ms) / 
                (prt2_ms / prt_ms - (int) (prt2_ms / prt_ms)));
        Y = (int) (X * prt2_ms / prt_ms);

        PrtScheme = (Y << 4) | X;
    } else {
        // Single prt
    	// PRT must be integral multiple of pulsewidth !
        periodCount = _prtCounts;
        PrtScheme = 0x0000;
    }

    std::cout << "periodCount is " << periodCount << std::endl;

    // Control Register
    P7142_REG_WRITE(BAR2Base + MT_ADDR, CONTROL_REG | ALL_SD3C_TIMER_BITS);

    // Enable Timer
    P7142_REG_WRITE(BAR2Base + MT_DATA, TIMER_ON);

    // Turn on Write Strobes
    P7142_REG_WRITE(BAR2Base + MT_WR, WRITE_ON);
    
    for (unsigned int i = 0; i < N_SD3C_TIMERS; i++) {
        std::cout << "Initializing timer " << i << ": delay " <<
            countsToTime(timerDelay(i)) << "s (" << timerDelay(i) <<
            "), width " << countsToTime(timerWidth(i)) << "s (" << 
            timerWidth(i) << ")" << (timerInvert(i)? ", inverted":"") << std::endl;
        
        // Delay Register
        // Address
        P7142_REG_WRITE(BAR2Base + MT_ADDR, DELAY_REG | SD3C_TIMER_BITS[i]);
        // Data
        P7142_REG_WRITE(BAR2Base + MT_DATA, timerDelay(i));

        // Pulse Width Register
        // Address
        P7142_REG_WRITE(BAR2Base + MT_ADDR, WIDTH_REG | SD3C_TIMER_BITS[i]);
        // Data
        P7142_REG_WRITE(BAR2Base + MT_DATA, timerWidth(i));
    }

    // All timers have identical configuration for period and multiple prt

    // Period Register
    // Address
    P7142_REG_WRITE(BAR2Base + MT_ADDR, PERIOD_REG | ALL_SD3C_TIMER_BITS);
    // Data
    P7142_REG_WRITE(BAR2Base + MT_DATA, periodCount);

    //Multiple PRT Register
    // Address
    P7142_REG_WRITE(BAR2Base + MT_ADDR, PRT_REG | ALL_SD3C_TIMER_BITS);
    // Data: Mult PRT Valu Timer 0
    P7142_REG_WRITE(BAR2Base + MT_DATA, PrtScheme);

    // Turn off Write Strobes
    P7142_REG_WRITE(BAR2Base + MT_WR, WRITE_OFF);

    return true;

}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::dataRate() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

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
        // per gate for even and 8 bytes per gate for odd pulses.
        rate = (int)((_prf/2)*(16+_gates*8*2)/_nsum);
        break;
    }

    return rate;
}

//////////////////////////////////////////////////////////////////////
ptime p7142sd3c::timeOfPulse(int64_t nPulsesSinceStart) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Figure out offset since transmitter start based on the pulse
    // number and PRT(s).
    double offsetSeconds;
    if (_staggeredPrt) {
        unsigned long prt1Count = nPulsesSinceStart / 2 + nPulsesSinceStart % 2;
        unsigned long prt2Count = nPulsesSinceStart / 2;
        offsetSeconds =  prt1Count /_prf + prt2Count / _prf2;
    } else {
        offsetSeconds = nPulsesSinceStart / _prf;
    }
    // Translate offsetSeconds to a boost::posix_time::time_duration
    double remainder = offsetSeconds;
    int hours = (int)(remainder / 3600);
    remainder -= (3600 * hours);
    int minutes = (int)(remainder / 60);
    remainder -= (60 * minutes);
    int seconds = (int)remainder;
    remainder -= seconds;
    int nanoseconds = (int)(1.0e9 * remainder);
    int fractionalSeconds = (int)(nanoseconds *
        (time_duration::ticks_per_second() / 1.0e9));
    time_duration offset(hours, minutes, seconds,
            fractionalSeconds);
    // Finally, add the offset to the _xmitStartTime to get the absolute
    // pulse time
    return(_xmitStartTime + offset);
}

//////////////////////////////////////////////////////////////////////
int64_t p7142sd3c::pulseAtTime(ptime time) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    // First get the time since transmit start, in seconds
    double timeSinceStart = 1.0e-6 * (time - _xmitStartTime).total_microseconds();
    
    // Now figure out the pulse number. Note that this should work even for
    // times before the radar start time (yielding negative pulse number).
    int64_t pulseNum = 0;
    if (_staggeredPrt) {
        // First count the complete pairs of PRT1 and PRT2
        double prt1 = 1.0 / _prf;
        double prt2 = 1.0 / _prf2;
        double pairTime = prt1 + prt2;
        pulseNum = 2 * int64_t(timeSinceStart / pairTime);
        // Then work with the remaining time that's a fraction of (PRT1 + PRT2)
        double remainingTime = fmod(timeSinceStart, pairTime);
        double absRemainingTime = fabs(remainingTime);
        int sign = (remainingTime < 0) ? -1 : 1;
        
        if (absRemainingTime > (prt1 + 0.5 * prt2)) {
            pulseNum += sign * 2;
        } else if (absRemainingTime > (0.5 * prt1)) {
            // If the remainder is greater than halfway between 0 and PRT1,
            // add 1 to the pulse count
            pulseNum += sign;
        }
    } else {
        double prt1 = 1.0 / _prf;
        pulseNum = int64_t(timeSinceStart / prt1);
        double remainingTime = fmod(timeSinceStart, prt1);
        int sign = (remainingTime < 0) ? -1 : 1;
        // If the remainder is more than 1/2 a PRT, add another pulse
        if (fabs(remainingTime) > (0.5 * prt1)) {
            pulseNum += sign;
        }
    }
    
    return(pulseNum);
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

} // end namespace Pentek
