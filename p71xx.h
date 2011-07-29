#ifndef P71XX_H_
#define P71XX_H_

#include "ptkdrv.h"
#include "ptkddr.h"
#include "math.h"
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <signal.h>

#ifdef OPT_428
#include "ptk7142_428.h"
#else
#include "ptk7142.h"
#endif

// Use the following reference for now, so that we
// get the 7142.h file from the ReadyFlow tree, rather than
// the one of the same name that is in the Pentek Linux driver
// tree.
/// @todo Remove readyflow/include once we have eliminated all of the
/// Pentek Linux driver code from the pentek access library.
#include "readyflow/include/7142.h"

#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {

	/// Base class for a p7142 digital transceiver card.
	///
    /// ReadyFlow seems to have two methods of configuration manipulation. At
    /// the highest level, a structure with fields corresponding
    /// to many of the GateFLow options is provided for each subsystem. The
    /// subsystems are PCI, DMA, board, input (A/D) and output (D/A).
	/// The user initializes a structure using a SetDefault() call,
	/// e.g. P7142SetPCIDefaults(). Individual fields in the structure are
	/// modified to configure for non-default behavior. The structure is then
	/// written to the board registers with an InitRegs() call, e.g.
	/// P7142InitPCIRegs().
	///
	/// ReadyFlow also provides get/set macros for manipulating individual
	/// control bits. So, once a section has had a complete initial configuration
	/// using InitRegs(), the  macros can be used to access individual
	/// control bits as needed. In fact, the InitRegs() functions are built
	/// around these macros.
	///
	/// The approach taken here will be to initialize the entire board in
	/// p71xx. The associated p7142Dn and p7142Up classes will use the
	/// ReadyFlow macros as needed.

	class p71xx {

		public:
			/// Constructor,
			/// @param devName The top level device name (e.g., 
            /// /dev/pentek/p7140/0. Other device names, such as ctrl will be 
            /// constructed as needed. The ctrl device will be opened and will 
            /// be available via the _ctrlFd file descriptor. Use ok() to 
            /// verify successful construction.
            /// @param simulate Set true if we operate in simulation mode.
			p71xx(std::string devName, bool simulate=false);
			/// Destructor.
			virtual ~p71xx();
			/// @return true if the last operation was successful,
			/// false otherwise.
			virtual bool ok() const;
			/// Return true iff we're in simulation mode.
			/// @return true iff we're in simulation mode.
			bool isSimulating() const { return _simulate; }

		protected:
			/// Initialize the ReadyFlow library.
			bool initReadyFlow();
            /// Return the file descriptor for the control device.
            /// @return the file descriptor for the control device.
            int ctrlFd() { return _ctrlFd; }
            /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @return The result of the ioctl call.
			static int doIoctl(int fd, int ioctlCode, void* arg, std::string errMsg, bool doexit=true);
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @return The result of the ioctl call.
			static int doIoctl(int fd, int ioctlCode, int arg, std::string errMsg, bool doexit=true);
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @return The result of the ioctl call.
			static int doIoctl(int fd, int ioctlCode, double arg, std::string errMsg, bool doexit=true);
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
            sem_t wdSemKey;
            void* hDev;
            DWORD intrStat;
            DWORD libStat;
            DWORD dmaBufStat;
            PTK714X_DMA_HANDLE* dmaHandle;
            PTK714X_DMA_BUFFER  dmaBuf;
            DWORD           BAR0Base;          /* PCI BAR0 base address */
            DWORD           BAR2Base;          /* PCI BAR2 base address */
            DWORD           slot;
            unsigned int    moduleId;
            P7142_REG_ADDR            p7142Regs;
            P7142_PCI_PARAMS          p7142PciParams;      /* PCI7142 PCI params */
            P7142_DMA_PARAMS          p7142DmaParams;      /* PCI7142 DMA params */
            P7142_BOARD_PARAMS        p7142BoardParams;    /* board params       */
            P7142_INPUT_PARAMS        p7142InParams;       /* input block params */
            P7142_OUTPUT_PARAMS       p7142OutParams;      /* output block params */
            int* adcData;
            /// Indicated the success of the last operation.
            bool _ok;
            /// The root device name
            std::string _devName;
            /// The ctrl device name
            std::string _devCtrl;
	        /// file descriptor for the ctrl device
	        int _ctrlFd;
            /// set true if in simulation mode
            bool _simulate;
            /// recursive mutex which provides us thread safety.
            mutable boost::recursive_mutex _mutex;
            /// Pointer to the sd3c revision register in the signal fpga.
            DWORD* svnRevReg;
            /// Pointer to the sd3c transceiver control register in the fpga.
            DWORD* tcvrCtrlReg;

	};
}

#endif /*P71XX_H_*/
