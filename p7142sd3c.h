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

namespace Pentek {

/// @brief A p7142 class specialized for cards running the SD3C firmware.
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
      DDCUNDEFINED, DDC10DECIMATE, DDC8DECIMATE, DDC6DECIMATE, DDC4DECIMATE, BURST
    } DDCDECIMATETYPE;

    /// @brief Constructor.
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
    ///     disabled. Note that this is the total number of beams which will go into
    ///     the even and odd sums; i.e. the even beam integration will collect
    ///     nsum/2 beams and the odd beam integration will collect nsum/2 beams.
    /// @param freeRun If true, the firmware will be configured to ignore the 
    ///     PRT gating.
    /// @param requiredDDCType the DDC type required to be in the Pentek's
    ///     firmware.
    /// @param externalStartTrigger If true, an external trigger source
    ///     (generally a 1 PPS signal from a GPS clock) is used to start the 
    ///     radar. This parameter used to have a default value of 
    ///     false, but a default is no longer defined.
    /// @param simPauseMS The number of milliseconds to wait between beams
    ///     simulated data when calling read(). This parameter used to
    ///     have a default value of 50, but a default is no longer 
    ///     defined.
    /// @param useFirstCard If true, use the first card in the system. Otherwise,
    ///     the next card will be searched for. This parameter used to
    ///     have a default value of false, but a default is no longer 
    ///     defined.
    /// @param rim If true, we are operating in RIM mode. This parameter
    ///     used to have a default value of false, but a default is no
    ///     longer defined.
    /// @param codeLength If complementary coding is being used, it is
    ///     set to the length of the code. This parameter used to have a
    ///     default of 0, but a default is no longer defined.
    /// @param adc_clock - if non-zero, override the default adc_clock 
    ///     from DDC Type. This parameter used to have a default of 0,
    ///     but a default is no longer defined.
    /// @param reset_clock_managers:
    ///     if true, do the following at the end of the constructor:
    ///       Reset the clock managers
    ///       set free run mode
    ///       and stop filters
    ///     This is the normal default situation.
    ///     However, if you have more than 1 pentek card and need to sync
    ///     the startup of the cards in time relative to each other,
    ///     set this to false and then call resetClockManagers() from the
    ///     client code just before starting the timers and the filters.
    ///     As an example, this is done in DowDrx, in TimerManager.cpp.
    /// @param abortOnError
    ///     Set true if you want the constructor to abort on error
    ///     Set false if you want the constructor to return on error
    ///     after setting _constructorOk to false
    /// @param sd3cTimerDivisor Divisor to use for SD3C's internal timer
    ///     clock w.r.t. the ADC clock. Allowed values are 2, 4, 8, and 16.
    ///     SD3C's internal timers use a clock divided down from the ADC clock.
    ///     Increasing this value reduces the time resolution of the SD3C
    ///     timers, but allows for larger maximum time intervals. (Maximum
    ///     timer interval allowed is 65535 * the timer's resolution.)
    p7142sd3c(
    		bool simulate,
    		double tx_delay,
    		double tx_pulsewidth,
    		double prt,
    		double prt2,
    		bool staggeredPrt,
    		unsigned int gates,
    		unsigned int nsum,
    		bool freeRun,
    		DDCDECIMATETYPE requiredDDCType,
    		bool externalStartTrigger,
    		double simPauseMS,
    		bool useFirstCard,
    		bool rim,
    		int codeLength,
    		double adc_clock,
    		bool reset_clock_managers = true,
    		bool abortOnError = true,
    		uint16_t sd3cTimerDivisor = 2
    		);
    
    /// Destructor.
    virtual ~p7142sd3c();
    
    /// @return True if the constructor was successful, false otherwise.
    /// Applies if abortOnError is true.
    virtual bool constructorOk() const { return _constructorOk; }

    /// @brief Construct and add a downconverter for one of our receiver channels.
    /// @param chanId The channel identifier (0-3)
    /// @param dmaDescSize Size of a DMA descriptor. This is the spacing, in
    ///     bytes, between user-space interrupts to read DMA memory.
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
    /// @param simWaveLength The wavelength of the simulated data, in sample 
    ///     counts
    /// @param internalClock Set to true if the Pentek card's internal clock
    ///     should be used
    /// return NULL on error
    virtual p7142sd3cDn * addDownconverter(
            int chanId, 
            uint32_t dmaDescSize,
            bool burstSampling,
            int tsLength,
            double rx_delay, 
            double rx_pulse_width,
            std::string gaussianFile, 
            std::string kaiserFile,
            int simWaveLength = 5000,
            bool internalClock = false);
    
    /// @return The sd3c firmware revision number.
    int sd3cRev() const;

    /// @brief Stop the DMA for a specified downconverter
    /// @param chan The desired channel
    void stopDMA(int chan);

    /// @brief Return the ADC clock frequency in Hz.
    /// @return The ADC clock frequency in Hz.
    double adcFrequency() const {
        return _adcClock;
    }
    
    /// @brief Convert the given time in seconds to the nearest integer SD3C
    /// internal timer counts.
    /// @ param time the time to be converted, in seconds
    /// @ return the nearest integer SD3C internal timer counts for the given
    /// time
    int timeToCounts(double time) const;
    
    /// @brief Convert a time in integer SD3C internal timer counts to
    /// a time in seconds.
    /// @ param time the time to be converted, in SD3C internal timer counts.
    /// @ return the equivalent time in seconds
    double countsToTime(int counts) const;
    
    /// @brief Start or stop the 8 SD3C timers.
    ///
    /// If starting the timers, actual timer
    /// start will occur at the next trigger event (which may be internal or
    /// external, depending on the setting of externalStartTrigger at 
    /// construction.
    /// @param start Set true to start, set false to stop.
    /// @return true if successful, false otherwise.
    bool timersStartStop(bool start);
    
    /// @brief Return the time of the first pulse sample.
    /// @return Time of first pulse sample
    boost::posix_time::ptime radarStartTime() const {
        return _radarStartTime;
    }

    /// @brief Return the first PRT, in seconds.
    /// @return The first PRT, in seconds
    double prt() const {
        return countsToTime(_prtCounts);
    }
    
    /// @brief Return the first PRT, in SD3C internal timer counts
    /// @return The first PRT, in SD3C internal timer counts
    uint32_t prtCounts() const {
        return _prtCounts;
    }
    
    /// @brief Return the second PRT, in seconds. Return zero if not
    /// running with staggered PRT.
    /// @return The second PRT, in seconds, or zero if not running staggered
    /// PRT.
    double prt2() const {
        return countsToTime(_prt2Counts);
    }
    
    /// @brief Return the second PRT in SD3C internal timer counts, or zero
    /// if not running staggered PRT.
    /// @return the second PRT in SD3C internal timer counts, or zero
    /// if not running staggered PRT.
    uint32_t prt2Counts() const {
        return(_staggeredPrt ? _prt2Counts : 0);
    }
    
    /// @brief Set the time of the first transmit pulse.
    /// @param startTime The boost::posix_time::ptime of the first transmit
    ///    pulse.
    void setXmitStartTime(boost::posix_time::ptime startTime) {
        _radarStartTime = startTime;
    }

    /// @brief Are we operating in range imaging mode?
    /// @return True if operating in RIM mode.
    bool rim();

    /// @brief Read the ttl input lines from the fpga
    /// @return The input line values.
    unsigned short int TTLIn();

    /// @brief Set the ttl output lines on the FPGA.
    /// @param data The value to be written.
    void TTLOut(unsigned short int data);

    /// @brief Return the firmware version number
    /// @return the firmware version number
    int firmwareVersion() const { return _firmwareVersion; }

    /// @brief Return the DDC type instantiated in our card's firmware
    /// @return the DDC type instantiated in our card's firmware
    DDCDECIMATETYPE ddcType() const { return(_ddcType); }

    /// @brief Return the DDC decimation factor associated with the currently
    /// loaded firmware.
    /// @return the DDC decimation factor associated with the currently
    /// loaded firmware.
    uint16_t ddcDecimation() const;
    
    /// @brief Return the name of the firmware DDC type
    /// @return the name of the firmware DDC type
    std::string ddcTypeName() const;
    
    /// @brief Return the name of the given DDCDECIMATETYPE
    /// @return the name of the given DDCDECIMATETYPE
    static std::string ddcTypeName(DDCDECIMATETYPE type);

    /// @brief Start the fifos and the filters.
    /// Enable the gate generator, which will enable all of the fen
    /// generators at the same time. This  is how the downconverters are all
    /// phase synchronized.Follow this with clear the filter stop bit,
    /// which starts the filter coefficient counters running.
    void startFilters();

    /// @brief Stop the fifos and filters.
    /// Set the filter stop bit, which stops the filter coefficient
    /// counters and resets them. Follow this with disabling the gate generator,
    /// which will disable all of the fen generators at the same time.
    void stopFilters();
    
    /// Reset the clock managers
    /// set free run mode
    /// and stop filters
    void resetClockManagers();

    /// @brief Return the transmit pulse width, in seconds
    /// @return the transmit pulse width, in seconds
    double txPulseWidth() const;
    
    /// @brief Return the transmit pulse width, in local counts, which are units of
    /// (2 / adc_freq) seconds.
    int txPulseWidthCounts() const;
    
    /// @brief Set up general purpose timer 0. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer0(double delay, double width, bool invert = false);
    
    /// @brief Set up general purpose timer 1. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer1(double delay, double width, bool invert = false);
    
    /// @brief Set up general purpose timer 2. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer2(double delay, double width, bool invert = false);
    
    /// @brief Set up general purpose timer 3. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    /// @param invert true if the timer output should be inverted
    void setGPTimer3(double delay, double width, bool invert = false);
    
    /// @brief Return the number of gates being sampled by our non-burst downconverters.
    /// @return the number of gates being sampled by our non-burst 
    ///     downconverters
    unsigned int gates() const;
    
    /// @brief Return the number of pulses to sum for coherent integration, used by
    /// all of our non-burst downconverters.
    ///
    /// It represents the number of
    /// beams which go into an even beam accumulation, and likewise the
    /// number of beams which go into an odd beam accumulation.
    /// If nsum == 1, coherent integration is disabled.
    unsigned int nsum() const;
    
    /// @return The length of the complimentary code. 0 means that the code
    /// is not being used.
    int codeLength() const;

    /// @brief Return the expected data bandwidth from a (non-burst) receiver channel
    /// in bytes per second
    /// @return The expected data bandwidth from a (non-burst) receiver channel 
    /// in bytes per second
    int dataRate();

    /// @brief Return the offset time of the given pulse, in seconds since
    /// the radar start time. This method is inlined because it is called
    /// *very* frequently.
    /// @param nPulsesSinceStart the pulse number for which the time is wanted
    /// @return the offset time of the given pulse, in seconds since
    /// the radar start time.
    inline double offsetSecondsOfPulse(uint64_t nPulsesSinceStart) const {
        // Figure out offset since transmitter start based on the pulse
        // number and PRT(s).
        //
        // NOTE: nPulsesSinceStart is 1-based

        double offsetSeconds;
        int64_t count = nPulsesSinceStart - 1;
        if (_staggeredPrt) {
          unsigned long prt2Count = count / 2;
          unsigned long prt1Count = prt2Count + count % 2;
          offsetSeconds = (prt1Count * _prt) + (prt2Count * _prt2);
        } else {
          offsetSeconds = count * _prt;
        }
        
        return(offsetSeconds);
    }
    
    ////////////////////////////////////////////////////////////////////////
    /// @brief Return the time of the given transmit pulse.
    /// @param nPulsesSinceStart the pulse number for which the time is wanted
    /// @return Time of the given transmit pulse.
    /// This method is inlined because it gets called a lot,
    /// and removing the call overhead helps things noticeably.

    inline boost::posix_time::ptime
      timeOfPulse(int64_t nPulsesSinceStart) const {

        // Get the offset since radar start.
        double offsetSeconds = offsetSecondsOfPulse(nPulsesSinceStart);

        // Convert subseconds to boost::posix_time::time_duration "ticks"
        double subseconds = fmod(offsetSeconds, 1.0);
        int fractionalSeconds = 
            (int)(subseconds * 
                  boost::posix_time::time_duration::ticks_per_second());

        // Now construct a boost::posix_time::time_duration from the
        // seconds and fractional seconds
        boost::posix_time::time_duration
          offset(0, 0, long(offsetSeconds), fractionalSeconds);

        // Finally, add the offset to the _radarStartTime to get the absolute
        // pulse time

        return(_radarStartTime + offset);

    }

    ////////////////////////////////////////////////////////////////////////
    /// @brief Return the time of the given transmit pulse, and set the values
    /// of the given references to the pulse's (integer) seconds since Epoch, 
    /// (integer) seconds since radar start time, and (integer) nanoseconds 
    /// into the second for each of those values.
    /// @param[in] nPulsesSinceStart the pulse number for which the time is wanted
    /// @param[out] secondsSinceEpoch pulse time in integer seconds since the
    /// Epoch
    /// @param[out] secondsSinceStart pulse time in integer seconds since radar
    /// start
    /// @param[out] nanoSeconds pulse time subsecond time, in nanoseconds since
    /// the top of the second
    /// @return Time of the given transmit pulse.

    inline boost::posix_time::ptime timeOfPulse(int64_t nPulsesSinceStart,
                                                int64_t &secondsSinceEpoch,
                                                int64_t &secondsSinceStart,
                                                int64_t &nanoSeconds) const
    {
      // Get the offset since radar start.
        
      double offsetSeconds = offsetSecondsOfPulse(nPulsesSinceStart);

      
      // Compute seconds and nanoseconds

      secondsSinceStart = int64_t(offsetSeconds);
      double subseconds = fmod(offsetSeconds, 1.0);
      nanoSeconds = int64_t(subseconds * 1.0e9 + 0.5);

      int fractionalSeconds = 
        (int)(subseconds *
              boost::posix_time::time_duration::ticks_per_second());
      
      // Now construct a boost::posix_time::time_duration from the
      // seconds and fractional seconds

      boost::posix_time::time_duration
        offset(0, 0, long(offsetSeconds), fractionalSeconds);
      
      // Finally, add the offset to the _radarStartTime to get the absolute
      // pulse time

      boost::posix_time::ptime pulseTime =
        _radarStartTime + offset;
      boost::posix_time::time_duration
        timeFromEpoch = pulseTime - Epoch1970;
      secondsSinceEpoch = timeFromEpoch.total_seconds();

      return pulseTime;

    }
    
    /// @brief Return the closest pulse number to a given time.
    /// @return The closest pulse number to a given time.

    int64_t pulseAtTime(boost::posix_time::ptime time) const;

    /// @brief Return true iff the zero position for motor quadrature counting
    /// has been set.
    /// @return true iff the zero position for motor quadrature counting
    /// has been set.
    bool motorZeroPositionSet() const; 

    /// @brief Momentarily set the "zero motor counts" bit in the TTL_OUT1
    /// register. This causes the firmware to zero the quadrature counts for all
    /// motors being monitored.
    void zeroMotorCounts();

    /// Epoch - 1970-01-01 00:00:00 UTC

    static const boost::posix_time::ptime Epoch1970;

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

    /// @brief Return the name for the selected SD3C timer index
    /// @param timerNdx the integer (or TimerIndex) index for the timer of
    ///    interest
    /// @return the name for the selected SD3C timer index
    const std::string timerName(int timerNdx) const;

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
    
    /**
     * Return timer enabled state for the selected timer. While
     * an integer index may be used explicitly, it is recommended to use
     * a TimerIndex enumerated value instead.
     * @param timerNdx the integer (or TimerIndex) index for the timer of
     *     interest
     * @return true if the timer is enabled
     */
    bool timerEnabled(int timerNdx) const;

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
    bool initTimers(bool isStart = false);

public:
    /// If _freerun is true, set the FREERUN bit in the
    /// transceiver control register. Otherwise clear it.
    void loadFreeRun();

    /// @brief Return the default ADC clock frequency for the given
    /// DDCDECIMATETYPE.
    /// @param ddcType the DDCDECIMATETYPE of interest
    /// @return the default ADC clock frequency for the given DDCDECIMATETYPE.
    static double DefaultAdcFrequency(DDCDECIMATETYPE ddcType);

protected:

    /**
     * Simple class to hold state of an SD3C timer
     */
    class _TimerConfig {
    public:
        _TimerConfig(int delay, int width, bool invert, bool enabled) :
           _delay(delay), 
           _width(width),
           _invert(invert),
           _enabled(enabled) {}
        _TimerConfig() : _delay(0), _width(0), _invert(false), _enabled(false) {}
        int delay() const { return _delay; }
        int width() const { return _width; }
        int invert() const { return _invert; }
        bool enabled() const { return _enabled; }
        void setEnabled(bool enabled) { _enabled = enabled; }
    private:
        int _delay;
        int _width;
        bool _invert;
        bool _enabled;
    };
    
    /// The three operating modes: free run, pulse tag, coherent integration, and coherent integration with RIM.
    typedef enum { MODE_FREERUN, MODE_PULSETAG, MODE_CI, MODE_CI_RIM} OperatingMode;
    
    /// Return our operating mode: free run, pulse tag and coherent integration.
    /// @return operating mode: free run, pulse tag and coherent integration.
    OperatingMode _operatingMode() const { return _mode; }
    
    /// @return The sd3c DDC type and software repository revision
    /// number, as read from the FPGA.
    unsigned int _sd3cTypeAndRev();

    /// @brief Unpack the DDC type instantiated in our card's firmware
    /// @return the DDC type instantiated in our card's firmware
    DDCDECIMATETYPE _unpackDdcType();

    /// @brief Unpack the SD3C firmware revision number.
    /// @return the unpacked sd3c firmware revision number.
    int _unpackSd3cRev();

    /// @brief Return the SD3C internal clock frequency in Hz.
    ///
    /// By default, the SD3C internal clock frequency is half of the ADC clock
    /// frequency, but this is further divided by the SD3C_CLOCK_DIVIDER
    /// specified at time of construction.
    /// @return the SD3C internal clock frequency in Hz.
    double _sd3cFrequency() const {
        return(_adcClock / _sd3cTimerDivisor);
    }

    /// @brief Return the nearest integer number of ADC clock frequency cycles
    /// in the given time.
    /// @param secs the time to be converted, in seconds
    /// @return the nearest integer number of ADC clock frequency cycles
    /// in the given time.
    int32_t _timeToAdcCounts(double secs) const {
        return(int32_t(rint(secs * _adcClock)));
    }

    /// @brief Return the time in seconds for the given number of ADC clock
    /// counts
    /// @param adcCounts the number of ADC counts to convert to time
    /// @return the time in seconds for the given number of ADC clock
    /// counts
    double _adcCountsToTime(int32_t adcCounts) const {
        return(adcCounts / _adcClock);
    }

    /// @brief Return the control register value for the selected timer based
    /// on values currently in _timerConfigs.
    /// @param timerIndex the integer (or TimerIndex) index for the timer of
    ///    interest
    /// @return the control register value for the selected timer based
    /// on values currently in _timerConfigs.
    uint16_t _timerControlRegData(int timerIndex) const;

    /// handling error behavior
    bool _abortOnError;  ///< set true if we abort on error rather than return
    bool _constructorOk; ///< true if the constructor was successful, false otherwise

    /// Pointer to the sd3c transceiver control register in the fpga.
    uint32_t* tcvrCtrlReg;

    /// Vector of delay/width pairs for our 8 SD3C timers
    _TimerConfig _timerConfigs[N_SD3C_TIMERS];
    /// radar PRT in sd3c_clock() counts
    unsigned int _prtCounts;
    /// second PRT of staggered PRT in sd3c_clock() counts
    unsigned int _prt2Counts;
    /// Staggered PRT flag. If true, both PRT values are used in staggered
    /// mode.
    bool _staggeredPrt;
    /// Free-run mode flag. If true, the firmware is be configured to ignore PRT 
    /// gating.
    bool _freeRun;
    /// Time of the first xmit pulse.
    boost::posix_time::ptime _radarStartTime;
    /// The adc clock rate in Hz
    double _adcClock;
    /// The prt in seconds
    double _prt;
    /// The dual prt in seconds
    double _prt2;
    /// The prf(s) in Hz
    double _prf;
    /// The dual prf in Hz.
    double _prf2;
    /// The number of gates to be sampled by all non-burst downconverters.
    unsigned int _gates;
    /// The number of pulses to sum for coherent integration by all non-burst
    /// downconverters.
    unsigned int _nsum;
    /// Firmware version number
    int _firmwareVersion;
    /// DDC type instantiated in our card's firmware
    DDCDECIMATETYPE _ddcType;
    /// The SD3C firmware revision number
    int _sd3cRev;
    /// The three operating modes: free run, pulse tag and coherent integration
    OperatingMode _mode;
    /// Does radar start wait for an external trigger?
    bool _externalStartTrigger;
    /// Are we operating in range imaging mode?
    bool _rim;
    /// The complimentary code length, if coding is being used
    int _codeLength;
    /// Has motor position counting zero position been set?
    bool _motorZeroPositionSet;
    /// SD3C clock divisor setting w.r.t. the ADC clock
    uint16_t _sd3cTimerDivisor;
};

}

#endif /* P7142SD3C_H_ */
