#ifndef P71XX_H_
#define P71XX_H_

#include <iostream>
#include <iomanip>
#define DUMP_BUF(addr_, bytes_) {\
std::cout << std::endl << __PRETTY_FUNCTION__  << ":" << __LINE__ << " " << bytes_ << " bytes" << std::endl; \
std::cout << std::hex; \
for (unsigned int i_ = 0; i_ < (unsigned int)bytes_; i_++) { \
	std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)(addr_[i_]) << " "; \
	if (!((i_+1) % 40)) { \
		std::cout << std::endl; \
	} \
} \
std::cout << std::dec << std::endl; \
}

#include "math.h"
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <signal.h>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>


#ifdef OPT_428
#include "ptk7142_428.h"
#else
#include "ptk7142.h"
#endif

#include "7142.h"

#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {

/// The size of the dma transfers from the Pentek to user space.
/// This determines the size of the buffers that the DMA transfers
/// are collected in, and the size of the pending read buffer.
#define DMABUFSIZE 65536

class p71xx;

/// This structure holds user data associated with each
/// dma channel. A pointer to this structure is registered
/// with WinDriver, and that pointer is delivered to the
/// dma handler for each dma transfer complete interrupt.
struct DmaHandlerData {
	int chan;
	Pentek::p71xx* p71xx;
};

	/// Foundation class for a p7142 digital transceiver card.
	/// Card configuration and interaction are managed via the
    /// ReadyFlow C API.
    ///
	/// <h1>Overview</h1>
	/// ReadyFlow uses WinDriver to provide access to the PCI
	/// interface. There are two activities: reading and writing to the
	/// Pentek PCI register space, and interacting with DMA transfers
	/// from the card.
	///
	/// The PCI address space is mapped directly to registers in the
	/// Pentek PCI FPGA and the signal FPGA. Controlling the Pentek functions
	/// in this manner logical, although fairly involved.
	///
	/// The DMA system functions by allocating a collection of DMA buffers,
	/// which will be directly written into by the Pentek card. When a transfer
	/// is completed, a dma "interrupt" is triggered, which asynchronously
	/// executes a C handler function in the user process space. The DMA
	/// is configured to loop continuously through 4 DMA buffers.
    ///
	/// A system of buffers and logic is used to capture the delivered data,
	/// buffer it through system slowdowns, and feed it out to data read requests.
	///
	/// <h1>The Buffer scheme</h1>
	/// Three buffer systems are used:
	/// @li The WinDriver DMA buffers
	/// @li A circular list of buffers for intermediate storage
	/// @li A pending read buffer, used to accumulate bytes to satisfy read requests.
	///
	/// @li Each DMA interrupt causes dmaIntHandler() to be entered.
	/// @li dmaIntHandler() calls p71xx::dmaInterrupt(), which transfers
	/// the data from the dma buffer to a buffer in the circular buffer list.
	/// @li p71xx::read() will attempt to copy bytes from the buffers on the circular
	/// buffer list to the pending read buffer. If the pending read buffer
	/// has enough bytes to satisfy the the request, they will be transferred to
	/// the caller buffer and the read will return.
	/// @li Otherwise, p71xx::read() will block until another buffer is
	/// added in the circular buffer list.
	///
	/// The circular buffer list allows for a large number of DMA transfers to
	/// be saved in times of heavy system load. (However, it's not clear that
	/// the DMA interrupts will even be delivered in times of heavy load).
	///
	/// The pending read buffer is not strictly necessary. read() requests
	/// could be filled incrementally out of the circular buffers. But the pending
	/// read buffer makes the book keeping much easier, and the logic simpler.
	/// If performance appears to be an issue, this could be one area to look at
	/// for a redesign.
	///
    /// <h1>Lineage</h1>
    /// The p71xx class was mostly adapted from the ReadyFlow example
    /// programs. Its C ancestry is obvious. There are many references
    /// to the DAC5686. Our Pentek cards are customized, where the
    /// DAC5686 was replaced by the DAC5687. We retained the DAC5686
    /// nomenclature so as to match the ReadyFlow usage.
    ///
	/// <h1>ReadyFlow</h1>
    /// ReadyFlow seems to have two methods of configuration manipulation. At
    /// the highest level, a structure with fields corresponding
    /// to many of the GateFLow options is provided for each subsystem. The
    /// subsystems are PCI, DMA, board, input (down conversion), output (up conversion)
    /// and DAC.
    ///
	/// The user initializes a structure using a SetDefault() call,
	/// e.g. P7142SetPCIDefaults(). Individual fields in the structure are
	/// modified to configure for non-default behavior. The structure is then
	/// written to the board registers with an InitRegs() call, e.g.
	/// P7142InitPCIRegs().
	///
	/// ReadyFlow also provides get/set macros for manipulating individual
	/// control bits. So, once a section has had a complete initial configuration
	/// using P7142InitInitRegs(), the  macros can be used to access individual
	/// control bits as needed. In fact, the InitRegs() functions are built
	/// around these macros.
	///
	/// The approach taken here will be to initialize the entire board in
	/// p71xx. The associated p7142Dn and p7142Up classes will use the
	/// ReadyFlow macros as needed.
	class p71xx {

		public:
			/// Constructor,
			/// @param The board number.
            /// @param simulate Set true if we operate in simulation mode.
			p71xx(int boardNum, int dmabufsize = DMABUFSIZE, bool simulate=false);
			/// Destructor.
			virtual ~p71xx();
			/// @return true if the last operation was successful,
			/// false otherwise.
			virtual bool ok() const;
			/// Return true iff we're in simulation mode.
			/// @return true iff we're in simulation mode.
			bool isSimulating() const { return _simulate; }
			/// This routine is called from the ReadyFlow dma interrupt handler
			/// adcDmaIntHandler(). It copies data from the dma interrupt buffer to the circular
			/// buffer list. According to the ReadyFlow notes, dma interrupts are disabled
			/// while dmaIntHandler() is executing, and so they will be disabled as well
			/// while we are in this routine.
            void adcDmaInterrupt(int chan);
			/// @todo Do we need to be calling these functions? It was done in the Linux driver and readyflow,
			/// but they don't seem to do anything.
            void enableGateGen();

            /// ReadyFlow PCI BAR2 base address.
            DWORD                 _BAR2Base;
		protected:
			/// Initialize the ReadyFlow library.
			bool initReadyFlow();
			/// Create a random number, with Gaussian distribution about a 
			/// selected mean and with selected standard deviation.
            /// Useful for simulation
            /// @param[in] mean mean value of the Gaussian distribution
            /// @param[in] stdDev standard deviation of the Gaussian distribution
            /// @return the generated random number
            static double gauss(double mean, double stdDev);
            /// Set the dma buffer and interrupt buffersize. The
            /// buffersize must be at least 2x the interrupt buffer size.
            /// Perhaps it should be even more?
            /// @param fd file descriptor
            /// @param intbufsize Interrupt buffer size.
            /// @param bufN The driver buffer will be this factor times intbufsize
            /// @return 0 on success, -1 on failure.
            static int bufset(int fd, int intbufsize, int bufN);
            /// Configure the board parameters, in p7142BoardParams
            void configBoardParameters();
            /// Configure the DMA parameters. The DMA buffering scheme is also
            /// allocated and configured here.
            void configDmaParameters();
            /// Configure the down conversion path parameters
            void configInParameters();
            /// Configure the up conversion path parameters
            void configOutParameters();
            /// Configure the DAC parameters, in p7142Dac5686Params
            void configDacParameters();
            /// start DMA for the specified DMA channel
            void start(int chan);
            /// stop the DMA for the specified DMA channel.
            void stop(int chan);
            /// Read bytes from the ADC channel. If no data are
            /// available, the thread will be blocked. The request will not
            /// return until the exact number of requested bytes can
            /// be returned.
            /// Note that multiple threads will be accessing this routine,
            /// can be individually blocked by the call.
            /// @param chan The channel (0-3).
            /// @param buf Buffer to receive the bytes..
            /// @param bytes The number of bytes.
            /// @return The number of bytes read. If an error occurs, minus
            /// one will be returned.
            int adcRead(int chan, char* buf, int bytes);
            /// ReadyFlow device descriptor.
            void* _deviceHandle;
            /// ReadyFlow dma handles, one per channel
            PTK714X_DMA_HANDLE*   dmaHandle[4];
            /// ReadyFlow dma buffer address pointers, in user space
            PTK714X_DMA_BUFFER    dmaBuf[4];
            /// ReadyFlow user data. A pointer to these will be passed into
            /// dmaIntHandler().
            DmaHandlerData        _dmaHandlerData[4];
            /// ReadyFlow PCI BAR0 base address.
            DWORD                 _BAR0Base;
            /// ReadyFlow PCI slot number.
            DWORD                 _pciSlot;
            /// ReadyFlow module identifier.
            unsigned int          _moduleId;
            /// ReadyFlow 7142 register addresses in PCI space.
            P7142_REG_ADDR        _p7142Regs;
            /// ReadyFlow parameters for PCI configuration.
            P7142_PCI_PARAMS      _p7142PciParams;
            /// ReadyFlow parameters for DMA configuration.
            P7142_DMA_PARAMS      _p7142DmaParams;
            /// ReadyFlow parameters for overall board configuration.
            P7142_BOARD_PARAMS    _p7142BoardParams;
            /// ReadyFlow parameters for the down conversion path configuration.
            P7142_INPUT_PARAMS    _p7142InParams;
            /// ReadyFlow parameters for the up conversion path configuration.
            P7142_OUTPUT_PARAMS   _p7142OutParams;
            /// ReadyFlow parameters for DAC configuration.
            P7142_DAC5686_PARAMS  _p7142Dac5686Params;

            /// The PCI address of the GateFlow gte generation control
            /// register. It is not clear that we need to even use this.
            volatile unsigned int *gateGenReg;


            /// The board number.
            int _boardNum;
            /// set true if in simulation mode
            bool _simulate;
            /// recursive mutex which provides us thread safety.
            mutable boost::recursive_mutex _p71xxMutex;
            /// True if device is opened and accessible
            bool _isReady;
            /// true if an AD channel is running
            bool _adcActive[4];
            /// A circular list of buffers, filled from the dma transfers.
            /// One list per channel.
            boost::circular_buffer<std::vector<char> > _circBufferList[4];
            /// A mutex used to control access to the circular buffer list,
            /// which is filled during DMA interrupts and emptied by
            /// read requests.
            boost::mutex _circBufferMutex[4];
            /// A condition variable that will activate when
            /// the circular buffer list is non-empty.
            boost::condition_variable _circBufferCond[4];
            /// This is a vector, one per channel, used for temporary
            /// storage to satisfy read requests. To avoid ongoing
            /// resizing, we allocate it to it's full length (2*_dmaBufSize).
            /// Bytes are added to the end of this buffer, and sucked out of
            /// the beginning to satisfy read requests.,
            /// _readBufAvail tracks how many bytes are available in the buffer.
            /// _readBufOut is the index of the next byte available in the buffer.
            std::vector<char> _readBuf[4];
            /// The number of bytes available in the _readBuf.
            int _readBufAvail[4];
            /// The next avaiable byte in _readBuf.
            unsigned int _readBufOut[4];
            /// The number bytes to be transferred in each DMA transaction. Note
            /// that each Pentek channel can make use of up to four DMA buffers,
            /// filling them in a round-robin fashion.
            int _dmaBufSize;
            /// The number in the dma interrupt chain for each channel.
            /// The chain is 4 buffers long, and then it wraps back.
            int _chainIndex[4];

	};
}

#endif /*P71XX_H_*/
