#ifndef P7142_H_
#define P7142_H_

#include "p71xx.h"
#include "p7142Dn.h"
#include "p7142Up.h"
#include "DDCregisters.h"

#include <iostream>
#include <string>
#include <vector>


namespace Pentek {
	class p7142Up;
	class p7142Dn;

	/// Base class for a p7142 digital transceiver card.
	class p7142 : public p71xx {

		public:
			/// Constructor.
			/// @param devName The top level device name (e.g.,
			/// /dev/pentek/p7142/0. Use ok() to verify successful construction.
			/// @param dmaBufferSize The size of the dma buffers. An interrupt will occur
		    /// for this number of bytes, for each down channel.
			/// @param simulate Set true for simulation mode.
			p7142(std::string devName, int dmaBufferSize = 65536, bool simulate=false);
			/// Destructor.
			virtual ~p7142();
            /// A P7142 card has 4 receive channels available.
            static const int P7142_NCHANNELS = 4;
            /// (Suggested) time to sleep after P7142 ioctl calls, in microseconds
            static const int P7142_IOCTLSLEEPUS = 100;
			/// Return the base device name for our P7142 card.
			/// @return the base device name.
			std::string devName() const { return _devName; }
			
            /// Construct and add a downconverter for one of our receiver channels.
            /// @param chanId The channel identifier (used to select /dn/*B)
            /// @param bypassdivrate The bypass divider (decimation) rate
            /// @param simulate Set true if we operate in simulation mode.
            /// @param simWaveLength The wave length, in timeseries points, for the
            /// simulated data. See p7142Dn::read().
            /// @param sim4bytes Create 4 byte instead of 2 byte integers when
            /// in simulation mode. This simulates the output of the coherent 
            /// integrator.
            virtual p7142Dn* addDownconverter(int chanId, int bypassdivrate = 1,
                    int simWavelength = 5000, bool sim4bytes = false);
            
            /// Construct and add an upconverter for our device.
            /// @param sampleClockHz The DAC sample clock in Hz
            /// @param ncoFreqHz The NCO frequency in Hz
            /// @param mode The DAC CONFIG2 coarse mixer mode (See DAC5687 Data Sheet)
            virtual p7142Up* addUpconverter(
                    double sampleClockHz, double ncoFreqHz, char mode);
            
            // We make our associated downconverter and upconverter classes 
            // friends so that they have access to BAR registers, etc.
            // methods, etc.
            friend class p7142Dn;
            friend class p7142Up;

		protected:
            /// Add (or replace) a downconverter on our list. If the 
            /// downconverter is associated with a channel for which we already
            /// have a downconverter, the existing downconverter for that
            /// channel will be deleted. This object assumes ownership of the 
            /// incoming downconverter.
            /// @param downconverter the downconverter to be added.
            void addDownconverter(p7142Dn* downconverter);
            
            /// Add (or replace) our upconverter. If we already have an
            /// upconverter, it will be deleted. This object assumes ownership 
            /// of the incoming upconverter.
            /// @param upconverter the upconverter to be added.
            void _addUpconverter(p7142Up* upconverter);
            
            /// Perform a FIOREGGET ioctl call to the control device for the 
            /// given address. The resulting value is returned.
            /// @param addr The address of the register to get.
            /// @return The value in the selected register.
            unsigned int _regget(unsigned int addr);

            /// Reset the digital clock managers on the FPGA. Necessary since
            /// some of the new DCMs we add in the firmware use the
            /// CLKFX output, which won't lock at startup. <em>Downconverters
            /// must call this method whenever they change their clock source
            /// via the CLKSRCSET ioctl!</em>
            void resetDCM();
            /// Borrowed shamelessly from dmem_dac.c in the readyflow examples
            ///
            /// Write data to the selected DDR memory bank, using DMA Channel 7.
            /// @param p7142Regs Pointer to the 7142 register addres table
            /// @param bank Memory bank to write to. Use defines P7142_DDR_MEM_BANK0,
            /// P7142_DDR_MEM_BANK1 or P7142_DDR_MEM_BANK2
            /// @param bankStartAddr Address in the bank at which to start reading
            /// @param bankDepth Number of bytes to write. Yes, BYTES
            /// @param dataBuf Nointer to the data buffer containing the data
            /// @param hDev The 7142 Device Handle
            /// @returns <br>
            /// 0 - successful <br>
            /// 1 - invalid bank number  <br>
            /// 2 - invalid start address  <br>
            /// 3 - bank depth extends past the end of the DDR bank <br>
            /// 4 - DMA channel failed to open <br>
            /// 5 - DMA buffer allocation failed <br>
            /// 6 - semaphore creation failed <br>
            /// 7 - semaphore wait timed out
            int ddrMemWrite (P7142_REG_ADDR* p7142Regs,
                             unsigned int    bank,
                             unsigned int    bankStartAddr,
                             unsigned int    bankDepth,
                             int32_t*   dataBuf,
                             PVOID           hDev);
            /// The down converters attached to this device
			std::vector<p7142Dn*> _downconverters;
			/// The upconverters attached to this device
			p7142Up* _upconverter;
	};

} // end namespace Pentek

#endif /*P7142_H_*/
