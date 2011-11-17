#ifndef P7142SD3C_H_
#define P7142SD3C_H_

#include "p7142.h"
#include "p7142sd3cDn.h"
#include <sys/types.h>
#include <sys/stat.h>

#include <string>
#include <vector>
#include <map>

#include <stdio.h>
#include <stdlib.h>

#include <boost/date_time/posix_time/posix_time.hpp>

static const double SPEED_OF_LIGHT = 2.99792458e8;  // m s-1

namespace Pentek {

/// A p7142 class specialized for cards running the SD3C firmware.
///
/// <h2>The SD3C firmware</h2>
/// The Pentek 7142 and the associated SD3C firmware support four 
/// receiver channels. The first pair of channels share receive timing 
/// characteristics, as does the second pair.
///
/// <h2>Simulation</h2>
/// For development and testing without the transceiver hardware, the p7142sd3c can
/// be configured to operate in simulation mode. In this case, the p7142Dn class
/// is configured for simulation as well, and its read() function will synthesize
/// simulated data. p7142sd3c will add sync words and tags as appropriate, depending on the
/// operating mode. Synchronization errors are randomly inserted when operating in
/// simulation mode.
///
class p7142sd3c : public p7142 {
public:
    /// The types of downconverters that can be instantiated in the SD3C Pentek 
    /// firmware.
    typedef enum {
        DDC10DECIMATE, DDC8DECIMATE, DDC4DECIMATE, BURST
    } DDCDECIMATETYPE;

    /// Constructor.
    /// @param boardNum The board number. Use ok() to verify successful construction.
    /// @param dmaBufferSize The size of the DMA buffers. One interrupt will occur for
    /// this many bytes.
    /// @param simulate Set true for simulation mode.
    /// @param tx_delay the delay for the tx pulse in seconds
    /// @param tx_pulsewidth the length of the transmit pulse in seconds
    /// @param prt The radar PRT in seconds
    /// @param prt2 The second PRT of a staggered PRT sequence in seconds
    /// @param staggeredPrt set true to use the second PRT for staggered PRT mode
    /// @param gates The number of gates to be sampled by all non-burst
    ///     downconverters
    /// @param nsum The number of pulses to sum for coherent integration by all
    ///     non-burst downconverters. If nsum == 1, coherent integration is 
    ///     disabled. Note that this is the number of beams which will go into
    ///     the even or odd sum; i.e. the even beam integration will collect
    ///     nsum beams and the odd beam integration will collect nsum beams.
    ///     Thus it is half of the effective PRF decimation rate. Profiler
    ///     configurations usually specify two times this value.
    /// @param freeRun If true, the firmware will be configured to ignore the 
    ///     PRT gating.
    /// @param simulateDDCType The DDC type to use when running in simulation
    ///     mode.
    /// @param externalStartTrigger If true, an external trigger source
    ///     (generally a 1 PPS signal from a GPS clock) is used to start the 
    ///     radar.
    p7142sd3c(
    		int boardNum,
    		int dmaBufferSize,
    		bool simulate,
    		double tx_delay,
    		double tx_pulsewidth,
    		double prt,
    		double prt2,
    		bool staggeredPrt,
    		unsigned int gates,
    		unsigned int nsum,
    		bool freeRun,
    		DDCDECIMATETYPE simulateDDCType,
    		bool externalStartTrigger = false);
    
    /// Destructor.
    virtual ~p7142sd3c();
    
    /// Construct and add a downconverter for one of our receiver channels.
    /// @param chanId The channel identifier (0-3)
    /// @param burstSampling Set true if burst sampling should be used for this 
    ///     channel. Burst sampling implies that gates will be as short as the 
    ///     card's sampling clock will allow. The rx_pulsewidth and the sampling
    ///     clock frequency will determine the number of gates sampled.
    /// @param tsLength The number of pulses in one time series. Used to set 
    ///     interrupt buffer size, so that we have reasonable responsiveness in 
    ///     the data stream.
    /// @param rx_delay the delay to the first rx gate in seconds
    /// @param rx_pulse_width The total pulse sampling time (for all gates) in 
    ///     seconds
    /// @param gaussianFile Name of the file containing the Gaussian
    ///     filter parameters
    /// @param kaiserFile Name of the file containing the Kaiser
    ///     filter parameters
    /// @param simPauseMS The number of milliseconds to wait between beams
    ///     simulated data when calling read()
    /// @param simWaveLength The wavelength of the simulated data, in sample 
    ///     counts
    /// @param internalClock Set to true if the Pentek card's internal clock
    ///     should be used
    virtual p7142sd3cDn * addDownconverter(
            int chanId, 
            bool burstSampling,
            int tsLength,
            double rx_delay, 
            double rx_pulse_width,
            std::string gaussianFile, 
            std::string kaiserFile,
            double simPauseMS = 0.1,
            int simWaveLength = 5000,
            bool internalClock = false);
    
    /// Stop the DMA for a specified downconverter
    /// @param chan The desired channel
    void stopDMA(int chan);

    /// @return The ADC clock frequency in Hz.
    double adcFrequency() const {
        return _adc_clock;
    }
    
    /// Convert a time in seconds to integer timer counts, which are in units
    /// of (2 / _adc_clock).
    /// @ param time the time to be converted, in seconds
    /// @ return the time in integer timer counts, which are in units of
    /// (2 / _adc_clock) seconds.
    int timeToCounts(double time) const;
    
    /// Convert a time in (2 / _adc_clock) integer timer counts to
    /// a time in seconds.
    /// @ param time the time to be converted, in (2 / _adc_clock) counts.
    /// @ return the time in seconds.
    double countsToTime(int counts) const;
    
    /// Start or stop the 8 SD3C timers. If starting the timers, actual timer
    /// start will occur at the next trigger event (which may be internal or
    /// external, depending on the setting of externalStartTrigger at 
    /// construction.
    /// @param start Set true to start, set false to stop.
    void timersStartStop(bool start);
    
    /// @return Time of first transmit pulse
    boost::posix_time::ptime xmitStartTime() const {
        return _xmitStartTime;
    }

    /// @return The first PRT, in seconds
    double prt() const {
        return countsToTime(_prtCounts);
    }
    
    /// @return The first PRT, in units of (2 / adcFrequency())
    unsigned int prtCounts() const {
        return _prtCounts;
    }
    
    /// @return The second PRT, in seconds, or zero if not running staggered
    /// PRT.
    double prt2() const {
        return countsToTime(_prt2Counts);
    }
    
    /// @return The second PRT, in units of (2 / adcFrequency()), or
    ///     zero if not running staggered PRT.
    unsigned int prt2Counts() const {
        return(_staggeredPrt ? _prt2Counts : 0);
    }
    
    /// Set the time of the first transmit pulse.
    /// @param startTime The boost::posix_time::ptime of the first transmit
    ///    pulse.
    void setXmitStartTime(boost::posix_time::ptime startTime) {
        _xmitStartTime = startTime;
    }
    /// Read the ttl input lines from the fpga
    /// @return The input line values.
    unsigned short int TTLIn();

    /// Set the ttl output lines on the FPGA.
    /// @param data The value to be written.
    void TTLOut(unsigned short int data);

    /// @return the DDC type instantiated in our card's firmware
    DDCDECIMATETYPE ddcType();
    
    /// @return the name of the firmware DDC type
    std::string ddcTypeName() const;
    
    /// @return the name of the given DDCDECIMATETYPE
    static std::string ddcTypeName(DDCDECIMATETYPE type);

    /// Set the filter start bit, which starts the data flow for all channels.
    void startFilters();

    /// Stop the filters
    void stopFilters();
    
    /// The transmit pulse width, in seconds
    /// @return the transmit pulse width, in seconds
    double txPulseWidth() const;
    
    /// Return the transmit pulse width, in local counts, which are units of 
    /// (2 / adc_freq) seconds.
    int txPulseWidthCounts() const;
    
    /// Set up general purpose timer 0. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer0(double delay, double width, bool invert = false);
    
    /// Set up general purpose timer 1. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer1(double delay, double width, bool invert = false);
    
    /// Set up general purpose timer 2. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer2(double delay, double width, bool invert = false);
    
    /// Set up general purpose timer 3. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    void setGPTimer3(double delay, double width, bool invert = false);
    
    /// Return the number of gates being sampled by our non-burst downconverters.
    /// @return the number of gates being sampled by our non-burst 
    ///     downconverters
    unsigned int gates() const;
    
    /// Return the number of pulses to sum for coherent integration, used by
    /// all of our non-burst downconverters. It represents the number of
    /// beams which go into an even beam accumulation, and likewise the
    /// number of beams which go into an odd beam accumulation.
    /// If nsum == 1, coherent integration is disabled.
    unsigned int nsum() const;
    
    /// @return The expected data bandwidth from a (non-burst) receiver channel 
    /// in bytes per second
    int dataRate();

    /// @return Time of the given transmit pulse.
    boost::posix_time::ptime timeOfPulse(int64_t nPulsesSinceStart) const;
    
    /// @return The closest pulse number to a given time.
    int64_t pulseAtTime(boost::posix_time::ptime time) const;
    
    friend class p7142sd3cDn;
    
protected:
    /**
     * SD3C has dedicated uses for the first four of its eight timers. Here 
     * we provide convenient names which map to the indices of the timers.
     *  (0) MASTER_SYNC_TIMER - This timer provides the trigger which starts the 
     *      process of transmitting and receiving a pulse. Delays in other timers
     *      are relative to this trigger.
     *  (1) RX_01_TIMER - While this timer is on, data are sampled for channels 0 and 1
     *  (2) TX_PULSE_TIMER - The transmitter fires while this timer is on
     *  (3) GP_TIMER_0 - This timer is not used internally, but is routed
     *      to an external pin. For HCR, profiler, and Ka, this timer is used
     *      for modulation of the transmit pulse.
     *  (4) RX_23_TIMER - While this timer is on, data are sampled for channels 2 and 3
     *  (5) GP_TIMER_1 - This timer is not used internally, but is routed
     *      to an external pin.
     *  (6) GP_TIMER_2 - This timer is not used internally, but is routed
     *      to an external pin.
     *  (7) GP_TIMER_3 - This timer is not used internally, but is routed
     *      to an external pin.
     */
    typedef enum {
        MASTER_SYNC_TIMER, // timer 0 is the master sync timer
        RX_01_TIMER,       // timer 1 is the rx timer for channels 0 and 1
        TX_PULSE_TIMER,    // timer 2 is the tx pulse timer
        GP_TIMER_0,        // timer 3 is not used internally, but is routed
                           // externally for general purpose use
        RX_23_TIMER,       // timer 4 is the rx timer for channels 2 and 3
        GP_TIMER_1,        // timer 5 is not used internally, but is routed
                           // externally for general purpose use
        GP_TIMER_2,        // timer 6 is not used internally, but is routed
                           // externally for general purpose use
        GP_TIMER_3,        // timer 7 is not used internally, but is routed
                           // externally for general purpose use
        N_SD3C_TIMERS      // The count of SD3C timers, i.e., 8
    } TimerIndex;
    
    /**
     * ID bits associated with the eight SD3C timers; these are used for
     * ioctl-s dealing with the timers.
     */
    static const unsigned int SD3C_TIMER_BITS[N_SD3C_TIMERS];
    /**
     * ALL_SD3C_TIMER_BITS is a bit mask for operations on all eight timers.
     */
    static const unsigned int ALL_SD3C_TIMER_BITS;

    /**
     * Return timer delay in counts for the selected timer. While
     * an integer index may be used explicitly, it is recommended to use
     * a TimerIndex enumerated value instead.
     * @param timerNdx the integer (or TimerIndex) index for the timer of
     *     interest
     * @return the timer delay in counts
     */
    int timerDelay(int timerNdx) const;
    
    /**
     * Return timer width in counts for the selected timer. While
     * an integer index may be used explicitly, it is recommended to use
     * a TimerIndex enumerated value instead.
     * @param timerNdx the integer (or TimerIndex) index for the timer of
     *     interest
     * @return the timer width in counts
     */
    int timerWidth(int timerNdx) const;
    
    /**
     * Return timer invert flag for the selected timer. While
     * an integer index may be used explicitly, it is recommended to use
     * a TimerIndex enumerated value instead.
     * @param timerNdx the integer (or TimerIndex) index for the timer of
     *     interest
     * @return true if the timer is inverted
     */
    bool timerInvert(int timerNdx) const;
    
    /// Set delay and width values for the selected timer. Note that values
    /// set here are not actually loaded onto the card until the timers are
    /// started with timersStartStop().
    /// @param ndx the TimerIndex for the timer to be set
    /// @param delay the delay in counts for the timer
    /// @param width the width in counts for the timer to be held on
    /// @param verbose set to true for verbose output
    /// @param invert set true to invert the timer output
    void setTimer(TimerIndex ndx, int delay, int width, bool verbose = true, bool invert = false);
    
    /// Load configured timer values onto the device.
    /// @return true if successful, false otherwise.
    bool initTimers();
    
    /// If _freerun is true, set the FREERUN bit in the
    /// transceiver control register. Otherwise clear it.
    void loadFreeRun();

    /// @return The sd3c firmware revision number.
    int sd3cRev();

    /// @return The sd3c DDC type and software repository revision
    /// number, as read from the FPGA.
    unsigned int sd3cTypeAndRev();

    /**
     * Simple class to hold integer delay and width for a timer.
     */
    class _TimerConfig {
    public:
        _TimerConfig(int delay, int width, bool invert) : 
           _delay(delay), 
           _width(width),
           _invert(invert) {}
        _TimerConfig() : _delay(0), _width(0), _invert(false) {}
        int delay() const { return _delay; }
        int width() const { return _width; }
        int invert() const { return _invert; }
    private:
        int _delay;
        int _width;
        bool _invert;
    };
    
    /// The three operating modes: free run, pulse tag and coherent integration
    typedef enum { MODE_FREERUN, MODE_PULSETAG, MODE_CI } OperatingMode;
    
    /// Return our operating mode: free run, pulse tag and coherent integration.
    /// @return operating mode: free run, pulse tag and coherent integration.
    OperatingMode _operatingMode() const { return _mode; }
    
    /// Pointer to the sd3c transceiver control register in the fpga.
    uint32_t* tcvrCtrlReg;

     /// Vector of delay/width pairs for our 8 SD3C timers
    _TimerConfig _timerConfigs[N_SD3C_TIMERS];
    /// radar PRT in _adc_clock/2 counts
    unsigned int _prtCounts;
    /// second PRT of staggered PRT in _adc_clock/2 counts
    unsigned int _prt2Counts;
    /// Staggered PRT flag. If true, both PRT values are used in staggered
    /// mode.
    bool _staggeredPrt;
    /// Free-run mode flag. If true, the firmware is be configured to ignore PRT 
    /// gating.
    bool _freeRun;
    /// Time of the first xmit pulse.
    boost::posix_time::ptime _xmitStartTime;
    /// The adc clock rate in Hz
    double _adc_clock;
    /// The prf(s) in Hz
    double _prf;
    /// The dual prf in Hz.
    double _prf2;
    /// The number of gates to be sampled by all non-burst downconverters.
    unsigned int _gates;
    /// The number of pulses to sum for coherent integration by all non-burst
    /// downconverters.
    unsigned int _nsum;
    /// DDC type instantiated in our card's firmware
    DDCDECIMATETYPE _ddcType;
    /// DDC type to use when simulating (default DDC8DECIMATE).
    DDCDECIMATETYPE _simulateDDCType;
    /// The SD3C firmware revision number
    int _sd3cRev;
    /// The three operating modes: free run, pulse tag and coherent integration
    OperatingMode _mode;
    /// Does radar start wait for an external trigger?
    bool _externalStartTrigger;
};

}

#endif /* P7142SD3C_H_ */
