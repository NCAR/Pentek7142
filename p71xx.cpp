#include "p71xx.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>


using namespace Pentek;

sem_t memWriteDmaSem;
sem_t memReadDmaSem;

/************************************************************************/
void memWriteDmaIntHandler(PVOID               hDev,
                        int                lintSource,
                        PVOID               pData,
                        PTK714X_INT_RESULT *pIntResult)
{
    sem_post(&memWriteDmaSem);
}


/************************************************************************
 Function: dmaReadIntHandler

 Description: This routine is an interrupt handler for the DMA Descriptor
              Finish interrupt for DMA Channel 0.

 Inputs:      hDev        - 7142 Device Handle
              lintSource  - Local interrupt source generating the
                            interrupt(0-15).  In this example, this
                            corresponds with dmaChannel.
              pIntResults - Pointer to the interrupt results structure

 Returns:     none

 Notes:       Fifo interrupts are cleared by the Kernel Device Driver.
              Fifo interrupts are enabled when this routine is executed.
************************************************************************/
void memReadDmaIntHandler(PVOID               hDev,
                       int                lintSource,
                       PVOID               pData,
                       PTK714X_INT_RESULT *pIntResult)
{
    sem_post(&memReadDmaSem);
}

/// This function is called by windriver each time a dma transfer is complete
/// from an ADC channel (dma channels 0-3).
/// The user data (pData) that is delivered with the interrupt contains a
/// pointer to an instance of p71xx. The p71xx::adcDmaInterrupt() method is called
/// to handle the actual processing of the dma transfer.
///
/// DMA interrupts are cleared by the Kernel Device Driver.
///
/// DMA interrupts are enabled when this routine is executed.
/// @param hDev The 7142 Device Handle
/// @param dmaChannel - DMA channel generating the interrupt(0-3)
/// @param pData - Pointer to user defined data
/// @param pIntResults - Pointer to the interrupt results structure
void adcDmaIntHandler(
		PVOID hDev,
		unsigned int dmaChannel,
		PVOID pData,
        PTK714X_INT_RESULT *pIntResult)
{

	// Cast the user data to DmaHandlerData*
	DmaHandlerData* dmaData = (DmaHandlerData*) pData;

	// Call the dmaInterrupt member function in the p71xx object
	dmaData->p71xx->adcDmaInterrupt(dmaChannel);
}

////////////////////////////////////////////////////////////////////////////////////////
p71xx::p71xx(int boardNum, int dmabufsize,  bool simulate):
_boardNum(boardNum),
_simulate(simulate),
_p71xxMutex(),
_isReady(false),
_dmaBufSize(dmabufsize)
{

	// dma buffer size must be a multiple of 4
	if ((_dmaBufSize % 4) || (_dmaBufSize <= 0)) {
		std::cout <<"DMA buffer size must be a positive  multiple of 4 bytes, " << _dmaBufSize << " was specified" << std::endl;
		abort();
	}

    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);
    // If we're simulating, things are simple...
	if (_simulate) {
		_isReady = true;
		return;
	}

	// Initialize buffers and flags.
	for (int chan = 0; chan < 4; chan++) {
		_adcActive[chan] = false;
		_readBufAvail[chan] = 0;
		_readBufOut[chan] = 0;
		_circBufferList[chan].set_capacity(100);
		_readBuf[chan].resize(2*_dmaBufSize);
	}

	// initialize ReadyFlow
	_isReady = initReadyFlow();

	return;
}

////////////////////////////////////////////////////////////////////////////////////////
p71xx::~p71xx() {

    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);

    for (int i = 0; i < 4; i++) {
    	P7142DmaAbort(&(_p7142Regs.BAR0RegAddr), 0);
    }

    /* cleanup for exit */
    PTK714X_DeviceClose(_deviceHandle);
    PTK714X_LibUninit();
    std::cout << "ReadyFlow closed" << std::endl;

    return;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p71xx::adcDmaInterrupt(int chan) {
	/// @todo this logic needs to be refactored so that there is no dynamic allocation
	/// happening during dma interrrupts.
	std::vector<char> data;
	data.resize(_dmaBufSize);

	/// Copy the data into the vector
	memcpy(&data[0], (char*)_adcDmaBuf[chan].usrBuf + _chainIndex[chan]*_dmaBufSize, _dmaBufSize);

	// lock access to the circular buffer and copy dma data to it
	{
		boost::lock_guard<boost::mutex> lock(_circBufferMutex[chan]);
		// put the vector in the circular buffer
		_circBufferList[chan].push_back(data);
	}

	// use the condition variable to tell data consumer (i.e. read())
	// that new data are avaialble.
	_circBufferCond[chan].notify_one();

	// move to the next buffer in the dma chain
	_chainIndex[chan]++;
	if (_chainIndex[chan] == 4) {
		_chainIndex[chan] = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////////
int
p71xx::adcRead(int chan, char* buf, int bytes) {

	// this is where it all happens

	// _pendingReadBuf[chan] has room for 2*_dmaBufSize, so a read request
	// cannot ask for more than half of this, since we may need to
	// append one of the data blocks from the circular buffer list
	// to fillBuffers[chan].
	assert(bytes <= _dmaBufSize);

	// Wait until there are enough bytes in _fillBuffers[chan]
	// to satisfy this request.
	while (_readBufAvail[chan] < bytes) {
		{
			// move unused data to the front of the buffer
			for (int i = 0; i < _readBufAvail[chan]; i++) {
				int j = _readBufOut[chan] + i;
				_readBuf[chan][i] = _readBuf[chan][j];
			}
			_readBufOut[chan] = 0;
			// block until we have at least one dma buffer available
			// in the circular buffer. Note that unique_lock
			// releases the lock when it goes out of scope.
			boost::unique_lock<boost::mutex> lock(_circBufferMutex[chan]);
			while (_circBufferList[chan].size() == 0) {
				_circBufferCond[chan].wait(lock);
			}
			// assert that we will not overrun _fillBuffer
			assert(_readBufAvail[chan]+_dmaBufSize <= 2*_dmaBufSize);

			// At least one buffer is available in the circular buffer,
			// Transfer the data to _readBuf
			for (int i = 0; i < _dmaBufSize; i++) {
				int j = _readBufAvail[chan] + i;
				_readBuf[chan][j] = _circBufferList[chan][0][i];
			}
			_readBufAvail[chan] += _dmaBufSize;

			//DUMP_BUF(_buffers[chan][0],  _dmaBufSize);

			// and remove it from the circular buffer
			_circBufferList[chan].pop_front();
		}
	}

	// copy requested bytes from _readBuf[chan] to the user buffer
	for (int i = 0; i < bytes; i++) {
		int j = _readBufOut[chan] + i;
		buf[i] = _readBuf[chan][j];
	}

	_readBufOut[chan]   += bytes;
	_readBufAvail[chan] -= bytes;

	//DUMP_BUF(buf, bytes);

	// assert that we don't have a math logic error
	assert(_readBufAvail[chan] >=0);

	return bytes;
}

////////////////////////////////////////////////////////////////////////////////////////
int p71xx::memWrite(int bank, int32_t* buf, int bytes) {

	int retval = bytes;

	int status = ddrMemWrite (
			&_p7142Regs,
            bank,
            0,
            bytes,
            (uint32_t*)buf,
            _deviceHandle);

	if (status != 0) {
		std::cout << "memory write failed, ddrMemWrite returned " << status << std::endl;
		retval = -1;
	}

	return retval;
}

////////////////////////////////////////////////////////////////////////////////////////
int p71xx::memRead(int bank, int32_t* buf, int bytes) {

	int retval = bytes;

	int status = ddrMemRead (
			&_p7142Regs,
            bank,
            0,
            bytes,
            (uint32_t*)buf,
            _deviceHandle);

	if (status != 0) {
		std::cout << "memory read failed, ddrMemRead returned " << status << std::endl;
		retval = -1;
	}

	return retval;


	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p71xx::initReadyFlow() {
	_pciSlot = -1;
    _deviceHandle = NULL;

    /* initialize the PTK714X library */
    DWORD dwStatus = PTK714X_LibInit();
    if (dwStatus != PTK714X_STATUS_OK)
    {
        std::cerr << "Failed to initialize the PTK714X library" << std::endl;
        return false;
    }

    /* Find and open a PTK714X device (by default ID) */
    _deviceHandle = PTK714X_DeviceFindAndOpen(&_pciSlot, &_BAR0Base, &_BAR2Base);
    if (_deviceHandle == NULL)
    {
        std::cerr << "Pentek 7142 device not found" << std::endl;
    }

    /* Initialize 7142 register address tables */
    P7142InitRegAddr (_BAR0Base, _BAR2Base, &_p7142Regs);

    /* check if module is a 7142 */
    P7142_GET_MODULE_ID(_p7142Regs.BAR2RegAddr.idReadout, _moduleId);
    if (_moduleId != P7142_MODULE_ID)
    {
        std::cerr << "Failed to identify a Pentek 7142 module." << std::endl;
        return false;
    }

    std::cout << "Pentek 7142 device";
    std::cout << std::hex << " BAR0: 0x" << (void *)_BAR0Base;
    std::cout << std::hex << " BAR2: 0x" << (void *)_BAR2Base;
    std::cout << std::dec;
    std::cout <<std::endl;

    /// @todo Although we follow the normal ReadyFlow protocol
    /// for configuring the DAC (P7142SetDac5687Defaults()
    /// followed by P7142InitDac5687Regs()),
    /// the DAC is completely reconfigured in p7142Up().
    /// MWe need to modify P7142SetDac5687Defaults() and
    /// P7142InitDac5687Regs() to perform the correct configuration,
    /// so that it can be pulled out of p7142Up().

    // Reset board registers to default values
    P7142ResetRegs (&_p7142Regs);

    /* Load parameter tables with default values */
    P7142SetPciDefaults    (&_p7142PciParams);
    P7142SetDmaDefaults    (&_p7142DmaParams);
    P7142SetBoardDefaults  (&_p7142BoardParams);
    P7142SetDdrMemDefaults (&_p7142MemParams);
    P7142SetInputDefaults  (&_p7142InParams);
    P7142SetOutputDefaults (&_p7142OutParams);
    P7142SetDac5687Defaults(&_p7142Dac5686Params);

    // Apply our adjustments
    configDmaParameters();
    configBoardParameters();
    configInParameters();
    configOutParameters();

    /* Write parameter table values to the 7142 registers */
    P7142InitPciRegs    (&_p7142PciParams,   &(_p7142Regs.BAR0RegAddr));
    P7142InitDmaRegs    (&_p7142DmaParams,   &(_p7142Regs.BAR0RegAddr));
    P7142InitBoardRegs  (&_p7142BoardParams, &(_p7142Regs.BAR2RegAddr));
    P7142InitDdrMemRegs (&_p7142MemParams,   &(_p7142Regs.BAR2RegAddr));
    P7142InitInputRegs  (&_p7142InParams,    &(_p7142Regs.BAR2RegAddr));
    P7142InitOutputRegs (&_p7142OutParams,   &(_p7142Regs.BAR2RegAddr));
    P7142InitDac5687Regs(&_p7142OutParams,   &_p7142Dac5686Params,   &(_p7142Regs.BAR2RegAddr));

    enableGateGen();

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configBoardParameters() {

    // Board customization

    _p7142BoardParams.busAMaster      = P7142_MSTR_CTRL_MASTER;
    _p7142BoardParams.busATermination = P7142_MSTR_CTRL_TERMINATED;

    _p7142BoardParams.busASelectClock = P7142_MSTR_CTRL_SEL_CLK_EXT_CLK;
    _p7142BoardParams.busAClockSource = P7142_MSTR_CTRL_CLK_SRC_SEL_CLK;

    _p7142BoardParams.busASelectSync  = P7142_MSTR_CTRL_SEL_SYNC_REGISTER;
    _p7142BoardParams.busASyncSource  = P7142_MSTR_CTRL_SYNC_SRC_SEL_SYNC;

    _p7142BoardParams.busASelectGate  = P7142_MSTR_CTRL_SEL_GATE_REGISTER;
    _p7142BoardParams.busAGateSource  = P7142_MSTR_CTRL_GATE_SRC_SEL_GATE;

    /* Bus B parameters */

    _p7142BoardParams.busBMaster      = P7142_MSTR_CTRL_MASTER;
    _p7142BoardParams.busBTermination = P7142_MSTR_CTRL_TERMINATED;

    _p7142BoardParams.busBSelectClock = P7142_MSTR_CTRL_SEL_CLK_EXT_CLK;
    _p7142BoardParams.busBClockSource = P7142_MSTR_CTRL_CLK_SRC_SEL_CLK;

    _p7142BoardParams.busBSelectSync  = P7142_MSTR_CTRL_SEL_SYNC_REGISTER;
    _p7142BoardParams.busBSyncSource  = P7142_MSTR_CTRL_SYNC_SRC_SEL_SYNC;

    _p7142BoardParams.busBSelectGate  = P7142_MSTR_CTRL_SEL_GATE_REGISTER;
    _p7142BoardParams.busBGateSource  = P7142_MSTR_CTRL_GATE_SRC_SEL_GATE;

    _p7142BoardParams.endianness = P7142_MISC_CTRL_ENDIANNESS_LE;

}
////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configDmaParameters() {

	// Perform all DAM related configuration. Note that the buffering
	// scheme is initialized here as well.

	int status;

	// -------------------------------------------------------------------------------//
	// Configure the ADC channels
	for (int chan = 0; chan < 4; chan++) {

		// open a DMA channel (required before we allocate the buffer)
		status = PTK714X_DMAOpen(_deviceHandle, chan, &_adcDmaHandle[chan]);
		if (status != PTK714X_STATUS_OK) {
			std::cerr << __PRETTY_FUNCTION__ << ": Unable to open DMA channel " << chan << std::endl;
			abort();
		}
		// allocate DMA buffers, one per channel. All descriptors
		// for the channel will use consecutive areas in this buffer
		status = PTK714X_DMAAllocMem(_adcDmaHandle[chan], _dmaBufSize*4, &_adcDmaBuf[chan], (BOOL)0);
		if (status != PTK714X_STATUS_OK) {
			std::cerr << __PRETTY_FUNCTION__ << ": Unable to allocate a DMA buffer for channel " << chan << std::endl;
			abort();
		}

		/* Abort any existing transfers */
		P7142DmaAbort(&(_p7142Regs.BAR0RegAddr), chan);

		/* Flush DMA channel buffer */
		P7142DmaFlush(&(_p7142Regs.BAR0RegAddr), chan);

		// set up channel parameters */
		P7142DmaChanSetup(&(_p7142DmaParams.dmaChan[chan]),
						  PCI7142_DMA_CMD_STAT_DMA_ENABLE,
						  PCI7142_DMA_CMD_STAT_DEMAND_MODE_ENABLE,
						  PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
						  512,              /* Max Burst Count */
						  0);              /* Transfer Interval Count */

		// configure four chained descriptors for each channel.
		/// @todo There is a cryptic note in Section 5.19 of the Pentek 7142
		/// operating manual which says the following about using the chain mode:
		/// <br>
		/// If you setup a DMA channel for a continuous data transfer (Chain
		/// bit D31 = 1 in all four Descriptors), you must ensure that the gating
		/// signal used for the transfer is not stopped before you stop
		/// the transfer or the channel may hang up.
		/// <br> They don't say what "hang up" means.
		for (int d = 0; d < 4; d++) {
			P7142DmaDescptrSetup(
			&(_p7142DmaParams.dmaChan[chan]),
			d,                                                     /* descriptor number */
			_dmaBufSize,                                           /* transfer count in bytes */
			PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,             /* DMA interrupt */
			PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_NEXT,               /* type of descriptor */
			(unsigned long)_adcDmaBuf[chan].kernBuf+d*_dmaBufSize);    /* buffer address */
		}

		/* flush FIFO */
		P7142FlushFifo(&(_p7142Regs.BAR2RegAddr.adcFifo[chan]),
					   &(_p7142InParams.adcFifo[chan]));

		/* Flush the CPU caches */
		PTK714X_DMASyncCpu(&_adcDmaBuf[chan]);

		// initialize the dma chain index
		_chainIndex[chan] = 0;

		// Initialize the data that will be delivered to the dma interrupt handler
		_adcDmaHandlerData[chan].chan  = chan;
		_adcDmaHandlerData[chan].p71xx = this;


	}

}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configInParameters() {

    /* Sync Bus select */
    _p7142InParams.inputSyncBusSel = P7142_SYNC_BUS_SEL_A;

	///@todo Currently using the GateFlow gating signal to
	/// enable/disable data flow. Not sure that this is necessary
    /// or even having any effect..

    //set the gate generator register pointers */
    if (_p7142InParams.inputSyncBusSel == P7142_SYNC_BUS_SEL_A)
        gateGenReg = (volatile unsigned int *)(_p7142Regs.BAR2RegAddr.gateAGen);
    else
        gateGenReg = (volatile unsigned int *)(_p7142Regs.BAR2RegAddr.gateBGen);

    // disable FIFO writes (set Gate in reset)
	P7142_SET_GATE_GEN(&gateGenReg, P7142_GATE_GEN_DISABLE);

    // set the down conversion FIFO parameters

    for (int adchan = 0; adchan < 4; adchan++) {
    	// select data packing mode.  It can be unpacked or time-packed.
		// The program define is located at the top of the program.
		//
		_p7142InParams.adcFifo[adchan].fifoPackMode = P7142_FIFO_ADC_PACK_MODE_TIME_PACK;

		// set the FIFO decimation.  This allows the input data rate to the
		// FIFO to be reduced.  It can be a value from 0 to 0xFFF.  Actual
		// decimation is this value plus one.

		// set to decimation by 1 here, but will almost always be modified by the user
		// to an appropriate value.
		_p7142InParams.adcFifoDecimation[adchan] = 0;

		// The FIFO Almost Full and Almost Empty levels are set to default
		// values for all programs.  The values shown here are the default
		// values and and are provided to show usage.  Their values must be
		// chosen to work with the DMA channel maximum burst count value.
		//
		_p7142InParams.adcFifo[adchan].fifoAlmostEmptyLevel = 512;
		_p7142InParams.adcFifo[adchan].fifoAlmostFullLevel  = 544;
    }

}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configOutParameters() {

    // Customize the up conversion path

    /// @todo Is the following correct for PLL usage? What do they mean by "bypass"?
    _p7142OutParams.dacClkSel = P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS;
    _p7142OutParams.dacPllVdd = P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE;
    _p7142OutParams.outputSyncBusSel = P7142_SYNC_BUS_SEL_A;
    // Following pack mode is used becasue we are putting I and Q into a 32 bit memory word.
    _p7142OutParams.dacFifo.fifoPackMode = P7142_FIFO_DAC_PACK_MODE_UNPACK;
    _p7142OutParams.dacFifo.fifoWordSwap = P7142_FIFO_CTRL_WORD_SWAP_DISABLE;

    // The FIFO Almost Full and Almost Empty levels are set to default
    // values for all programs.  The values shown here are the default
    // values for DAC FIFOs and are provided to show usage.   Their values
    // must be chosen to work with the DMA channel maximum burst count
    // value.
    _p7142OutParams.dacFifo.fifoAlmostEmptyLevel = 6144;
    _p7142OutParams.dacFifo.fifoAlmostFullLevel  = 6176;
}

////////////////////////////////////////////////////////////////////////////////////////

void
p71xx::start(int chan) {

	if (chan < 0 || chan > 3) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ":" << " trying to start illegal adc channel "
				<< chan << std::endl;
				return;
	}

	if (!_isReady) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ":" << " trying to start sdc channel " << chan
				<< " while the card is not ready" << std::endl;
				return;
	}

	if (_adcActive[chan]) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ":" << " trying to start sdc channel " << chan
				<< " when it is already active" << std::endl;
				return;
	}


	// apply the parameters to the DMA registers for this DMA channel
	P7142DmaChanInit(&(_p7142DmaParams.dmaChan[chan]),
					 &(_p7142Regs.BAR0RegAddr),
					 chan);

	// enable the FIFO
	P7142_SET_FIFO_CTRL_FIFO_ENABLE(
		_p7142Regs.BAR2RegAddr.adcFifo[chan].FifoCtrl,
		P7142_FIFO_ENABLE);

	// enable the DMA interrupt, on descriptor finish.
	// _dmaHandlerData contains a pointer to "this", as well
	// as other details.
	int status = PTK714X_DMAIntEnable(_adcDmaHandle[chan],
								   PTK714X_DMA_DESCRIPTOR_FINISH,
								   &_adcDmaHandlerData[chan],
								   (PTK714X_INT_HANDLER)adcDmaIntHandler);

	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ": Unable to enable DMA interrupt for channel " << chan << std::endl;
		abort();
	}

	std::cout << "DMA interrupt enabled for channel " << chan << std::endl;

	// Enable the DMA. Transfers will not occur however until the GateFlow
	// FIFOs start receiveing data, which will take place when the sd3c
	// timers are started.
	P7142DmaStart(&(_p7142Regs.BAR0RegAddr), chan);

	// Mark this channel as active.
	_adcActive[chan] = true;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p71xx::enableGateGen() {
	// enable FIFO writes (release Gate from reset)

	///@todo Temporarily using the GateFlow gating signal to
	/// enable/disable data flow. This will be changed to
	/// using the timers for this purpose.
	/// @todo It doesn't seem to make any difference whether the gate is on
	/// or off. Need to track this down.
	P7142_SET_GATE_GEN(&gateGenReg, P7142_GATE_GEN_ENABLE);
	std::cout << "GateGen enabled" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p71xx::stop(int chan) {

	if (chan < 0 || chan > 3) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ":" << " trying to stop illegal adc channel "
				<< chan << std::endl;
		return;
	}

	if (!_isReady) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ":" << " trying to stop sdc channel " << chan
				<< " while the card is not ready" << std::endl;
		return;
	}

	if (!_adcActive[chan]) {
		return;
	}

	int status;

	/* Abort any existing transfers */
    P7142DmaAbort(&(_p7142Regs.BAR0RegAddr), chan);

	/* Disable DMA Interrupt for this channel */
   status = PTK714X_DMAIntDisable(_adcDmaHandle[chan]);
	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ <<
				": DMA interrupt disable failed" << std::endl;
	}

	status = PTK714X_DMAFreeMem(_adcDmaHandle[chan], &_adcDmaBuf[chan]);
	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ <<
		   ": DMA memory free failed" << std::endl;
	}

	status = PTK714X_DMAClose(_deviceHandle, _adcDmaHandle[chan]);
	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ <<
				": DMA channel close failed" << std::endl;
	}

	_adcActive[chan] = false;

	std::cout << "DMA terminated for ADC channel " << chan << std::endl;

}

////////////////////////////////////////////////////////////////////////////////////////
bool
p71xx::ok() const {
	return _isReady;
}

/////////////////////////////////////////////////////////////////////
double
p71xx::gauss(double mean, double stdDev) {

	// create a normally distributed random number,
	// using this nifty little algorithm.

	double x = rand()/(1.0*RAND_MAX);
	double y = rand()/(1.0*RAND_MAX);
	double u = sqrt(-2.0*log10(x))*cos(2.0*M_PI*y);

	// set the mean std deviation
	return stdDev * u + mean;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p71xx::bufset(int fd, int intbufsize, int bufN) {
	/// @todo The dma buffersize should be adjusted based on
	/// a specified desired dma interrupt rate. It will be up
	/// to the user to figure out what that should be, usually
	/// in terms of the desired number of beams per interrupt.
	return 0;
}

/****************************************************************************
 Function: ddrMemRead

 Description: reads data from the selected DDR memory bank using the
              DMA Channel 8.

 Inputs:      p7142Regs     - pointer to the 7142 register addres table
              bank          - use defines:
                                  P7142_DDR_MEM_BANK0
                                  P7142_DDR_MEM_BANK1
                                  P7142_DDR_MEM_BANK2
              bankStartAddr - address in the bank start reading
              bankDepth     - number bytes to read
              dataBuf       - pointer to the data buffer to store read data
              hDev          - 7142 Device Handle

 Returns:     0 - successful
              1 - invalid bank number
              2 - invalid start address
              3 - bank depth extends past the end of the DDR bank
              4 - DMA channel failed to open
              5 - DMA buffer allocation failed
              6 - semaphore creation failed
              7 - semaphore wait timed out
****************************************************************************/
int p71xx::ddrMemRead (P7142_REG_ADDR *p7142Regs,
                unsigned int    bank,
                unsigned int    bankStartAddr,
                unsigned int    bankDepth,
                unsigned int   *dataBuf,
                PVOID           hDev)

{
    PTK714X_DMA_HANDLE  *dmaReadHandle;
    PTK714X_DMA_BUFFER   dmaBuf;
    P7142_FIFO_PARAMS    fifoParams;   /* FIFO params */
    P7142_DMA_PARAMS     dmaParams;    /* DMA params */
    P7142_DDR_MEM_PARAMS ddrMemParams; /* DDR memory params */
    int                  status;


    /* check input parameters -------------------------------------------- */

    /* check bank */
    if ( (bank != P7142_DDR_MEM_BANK0) &&
         (bank != P7142_DDR_MEM_BANK1) &&
         (bank != P7142_DDR_MEM_BANK2)    )
        return (1);

    /* check bankStartAddr and bankDepth */
    if (bankStartAddr > P7142_DDR_MEM_BANK_BYTE_SIZE)
        return (2);
    if ( (bankStartAddr+bankDepth) > P7142_DDR_MEM_BANK_BYTE_SIZE)
        return (3);


    /* DMA setup --------------------------------------------------------- */

    /* open a DMA channel to read from the delay memory */
    status = PTK714X_DMAOpen(hDev, P7142_DMA_CHAN_8, &dmaReadHandle);
    if (status != PTK714X_STATUS_OK)
        return (4);

    /* allocate system memory for read data buffer */
    status = PTK714X_DMAAllocMem(dmaReadHandle, (bankDepth + 128), &dmaBuf, (BOOL)0);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAClose(hDev, dmaReadHandle);
        return (5);
    }


    /* Interrupt & semaphore setup --------------------------------------- */
    /* enable the DMA interrupt */
    status = PTK714X_DMAIntEnable(dmaReadHandle,
                                  PTK714X_DMA_DESCRIPTOR_FINISH,
                                  NULL, (PTK714X_INT_HANDLER)memReadDmaIntHandler);
    if (status != PTK714X_STATUS_OK)
        {
        PTK714X_DMAFreeMem(dmaReadHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaReadHandle);
        return (6);
        }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&memReadDmaSem,0,0))<0)
        {
        PTK714X_DMAIntDisable(dmaReadHandle);
        PTK714X_DMAFreeMem(dmaReadHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaReadHandle);
        return (7);
        }


    /* FIFO setup -------------------------------------------------------- */

    /* set FIFO parameter table to default values */
    P7142SetFifoDefaults(&fifoParams, P7142_FIFO_TYPE_DDR_MEM);

    /* flush the FIFOs */
    P7142FlushFifo (
        (P7142_FIFO_CTRL_REG_ADDR *)&(p7142Regs->BAR2RegAddr.ddrMemReadFifo),
        &fifoParams);

    /* enable the FIFO */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                    \
        p7142Regs->BAR2RegAddr.ddrMemReadFifo.FifoCtrl, \
        P7142_FIFO_ENABLE);


    /* DMA channel setup ------------------------------------------------- */

    /* Load parameter table with default values */
    P7142SetDmaDefaults (&dmaParams);

    /* set up channel parameters */
    P7142DmaChanSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_8]),
                      PCI7142_DMA_CMD_STAT_DMA_ENABLE,
                      PCI7142_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                      PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
                      512,             /* Max Burst Count */
                      0);

    /* setup descriptor parameters */
    P7142DmaDescptrSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_8]),
                         P7142_DMA_DESCPTR_0,
                         (bankDepth + 128),  /* Transfer Count bytes */
                         PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                         PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                         (unsigned long)dmaBuf.kernBuf);

    /* apply the parameters to the DMA registers */
    P7142DmaChanInit(&(dmaParams.dmaChan[P7142_DMA_CHAN_8]),
                     &(p7142Regs->BAR0RegAddr),
                     P7142_DMA_CHAN_8);

    /* Flush DMA channel buffer */
    P7142DmaFlush(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_8);


    /* DDR memory setup ------------------------------------------------ */

    /* load parameter tables with default values
     *
     * note: The DDR memory registers were set to default values in main()
     *       by the call to P7142ResetRegs().  This routine does NOT restore
     *       the registers to default values on exit.
     */
    P7142SetDdrMemDefaults(&ddrMemParams);

    /* set DDR Memory parameters - these are set based on program mode.
     * They select DDR memory input source and output destination, enable
     * the bank, etc.  The parameters are:
     *     ddrMemCtrlRdWrFifoBankSel - bank select
     *     ddrMemCtrlBankDir         - data direction
     *     ddrMemCtrlBank0Enable     - bank 0 enable
     *     ddrMemCtrlBank1Enable     - bank 1 enable
     *     ddrMemCtrlBank2Enable     - bank 2 enable
     *     ddrMemCtrlDacSource       - DAC data source
     *     ddrMemCtrlBank0Pack       - bank bank 0 packing mode
     *     ddrMemCtrlBank1Pack       - bank bank 1 packing mode
     * Rather that setting them individually, they will be set using the
     * ReadyFlow library function, P7142SetDdrMemCtrlParams().  Bank
     * start and bank depth (in 32-bit words) are also set.
     */
    switch (bank)
    {
        case P7142_DDR_MEM_BANK0:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_0_READ_MODE);
            ddrMemParams.ddrMemBank0StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank0Depth = /*P7142_DDR_MEM_BANK_MAX_DEPTH;*/(bankDepth/4);

                        ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth     = 0;

            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth     = 0;
        break;

        case P7142_DDR_MEM_BANK1:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_1_READ_MODE);
            ddrMemParams.ddrMemBank1StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank1Depth = /*P7142_DDR_MEM_BANK_MAX_DEPTH;*/(bankDepth/4);

                        ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth     = 0;

            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth     = 0;

        break;

        case P7142_DDR_MEM_BANK2:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_2_READ_MODE);
            ddrMemParams.ddrMemBank2StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank2Depth = /*P7142_DDR_MEM_BANK_MAX_DEPTH;*/(bankDepth/4);

                        ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth     = 0;

            ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth     = 0;
        break;
    }

    /* apply the parameter table to the registers */
    P7142InitDdrMemRegs (&ddrMemParams, &(p7142Regs->BAR2RegAddr));

    /* for debug: flood buffer with known pattern */
    memset (dmaBuf.usrBuf, 0xA5, bankDepth);


    /* start the transfer, wait for completion --------------------------- */

    /* read from DDR Memory Bank */
    P7142DmaStart(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_8);

    /* wait for interrupt completion */
    status = sem_wait(&memReadDmaSem);
    if (status != 0)
    {
        sem_destroy(&memReadDmaSem);
        PTK714X_DMAIntDisable(dmaReadHandle);
        PTK714X_DMAFreeMem(dmaReadHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaReadHandle);
        return (8);
    }

    /* Sync Io Caches*/
    PTK714X_DMASyncIo(&dmaBuf);

    /* copy DMA buffer to data buffer */
    memcpy (dataBuf, dmaBuf.usrBuf, bankDepth);


    /* clean up and exit ------------------------------------------------- */

    /* disable DDR memory */
    P7142_SET_DDR_MEM_MODE(p7142Regs->BAR2RegAddr.ddrMem.ddrMemCtrl,   \
                           P7142_DDR_MEM_DISABLE_MODE);

    /* FIFO disable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                                   \
        p7142Regs->BAR2RegAddr.ddrMemReadFifo.FifoCtrl,                \
        P7142_FIFO_DISABLE);

    /* clean up */
    sem_destroy(&memReadDmaSem);
    PTK714X_DMAIntDisable(dmaReadHandle);
    PTK714X_DMAFreeMem(dmaReadHandle, &dmaBuf);
    PTK714X_DMAClose(hDev, dmaReadHandle);

    return (0);
}


/****************************************************************************
 Function: ddrMemWrite

 Description: writes data to the selected DDR memory bank.  Uses DMA
              Channel 7.

 Inputs:      p7142Regs     - pointer to the 7142 register addres table
              bank          - use defines:
                                  P7142_DDR_MEM_BANK0
                                  P7142_DDR_MEM_BANK1
                                  P7142_DDR_MEM_BANK2
              bankStartAddr - address in the bank at which to start reading
              bankDepth     - number bytes to write
              dataBuf       - pointer to the data buffer containing the data
              hDev          - 7142 Device Handle

 Returns:     0 - successful
              1 - invalid bank number
              2 - invalid start address
              3 - bank depth extends past the end of the DDR bank
              4 - DMA channel failed to open
              5 - DMA buffer allocation failed
              6 - semaphore creation failed
              7 - semaphore wait timed out
****************************************************************************/
int p71xx::ddrMemWrite (P7142_REG_ADDR *p7142Regs,
                 unsigned int    bank,
                 unsigned int    bankStartAddr,
                 unsigned int    bankDepth,
                 unsigned int   *dataBuf,
                 PVOID           hDev)
{
    PTK714X_DMA_HANDLE  *dmaWriteHandle;
    PTK714X_DMA_BUFFER   dmaBuf;
    P7142_FIFO_PARAMS    fifoParams;   /* FIFO params */
    P7142_DMA_PARAMS     dmaParams;    /* DMA params */
    P7142_DDR_MEM_PARAMS ddrMemParams; /* DDR memory params */
    unsigned int         bankEndAddr = ((bankStartAddr + bankDepth) / 4);
    int                  status;
    unsigned int         lastAddr;
    unsigned int         overWriteVal;


    /* check input parameters -------------------------------------------- */

    /* check bank */
    if ( (bank != P7142_DDR_MEM_BANK0) &&
         (bank != P7142_DDR_MEM_BANK1) &&
         (bank != P7142_DDR_MEM_BANK2)    )
        return (1);

    /* check bankStartAddr and bankDepth */
    if (bankStartAddr > P7142_DDR_MEM_BANK_BYTE_SIZE)
        return (2);
    if ( (bankStartAddr+bankDepth) > P7142_DDR_MEM_BANK_BYTE_SIZE)
        return (3);


    /* DMA setup --------------------------------------------------------- */

    /* open a DMA channel to write to the delay memory */
    status = PTK714X_DMAOpen(hDev, P7142_DMA_CHAN_7, &dmaWriteHandle);
    if (status != PTK714X_STATUS_OK)
        return (4);

    /* allocate system memory for write data buffer */
    status = PTK714X_DMAAllocMem(dmaWriteHandle, bankDepth, &dmaBuf, (BOOL)0);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAClose(hDev, dmaWriteHandle);
        return (5);
    }

    /* copy data buffer to DMA buffer */
    memcpy (dmaBuf.usrBuf, dataBuf, bankDepth);


    /* Interrupt & semaphore setup --------------------------------------- */

    /* enable the DMA interrupt */
    status = PTK714X_DMAIntEnable(dmaWriteHandle,
                                  PTK714X_DMA_DESCRIPTOR_FINISH,
                                  NULL, (PTK714X_INT_HANDLER)memWriteDmaIntHandler);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaWriteHandle);
        return (6);
    }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&memWriteDmaSem,0,0))<0)
        {
        PTK714X_DMAIntDisable(dmaWriteHandle);
        PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaWriteHandle);
        return (7);
        }


    /* FIFO setup -------------------------------------------------------- */

    /* set FIFO parameter table to default values */
    P7142SetFifoDefaults(&fifoParams, P7142_FIFO_TYPE_DDR_MEM);

    /* flush the FIFOs */
    P7142FlushFifo (
        (P7142_FIFO_CTRL_REG_ADDR *)&(p7142Regs->BAR2RegAddr.ddrMemWriteFifo),
        &fifoParams);

    /* enable the FIFO */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                     \
        p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl, \
        P7142_FIFO_ENABLE);


    /* DMA channel setup ------------------------------------------------- */

    /* Flush DMA channel buffer */
    P7142DmaFlush(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_7);

    /* Load parameter table with default values */
    P7142SetDmaDefaults (&dmaParams);

    /* set up channel parameters */
    P7142DmaChanSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                      PCI7142_DMA_CMD_STAT_DMA_ENABLE,
                      PCI7142_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                      PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
                      2048,              /* Max Burst Count */
                      0);

    /* disable DAC buffering.  This parameter is not set by the above
     * function and is only required for DMA writes to the delay memory
     * when DMA Channel 6 or 7 is used.
     */
    dmaParams.dmaChan[P7142_DMA_CHAN_7].dmaDacBuffering =
        PCI7142_DMA_CMD_STAT_DAC_BUFFERING_DISABLE;

    /* setup descriptor parameters */
    P7142DmaDescptrSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                         P7142_DMA_DESCPTR_0,
                         bankDepth,  /* Transfer Count bytes */
                         PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                         PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                         (unsigned long)dmaBuf.kernBuf);

    /* apply the parameters to the DMA registers */
    P7142DmaChanInit(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                     &(p7142Regs->BAR0RegAddr),
                     P7142_DMA_CHAN_7);

    /* remap DMA channel for DDR Memory writes */
    PCI7142_SET_LCL_DMA7_OUT_ADDR(             \
        p7142Regs->BAR0RegAddr.lclDmaOutRemap, \
        PCI7142_DMA7_MAP_DDR_MEM_WR_FIFO);


    /* DDR memory setup ------------------------------------------------ */

    /* load parameter tables with default values
     *
     * note: The DDR memory registers were set to default values in main()
     *       by the call to P7142ResetRegs().  This routine does NOT restore
     *       the registers to default values on exit.
     */
    P7142SetDdrMemDefaults(&ddrMemParams);

    /* set DDR Memory parameters - these are set based on program mode.
     * They select DDR memory input source and output destination, enable
     * the bank, etc.  The parameters are:
     *     ddrMemCtrlRdWrFifoBankSel - bank select
     *     ddrMemCtrlBankDir         - data direction
     *     ddrMemCtrlBank0Enable     - bank 0 enable
     *     ddrMemCtrlBank1Enable     - bank 1 enable
     *     ddrMemCtrlBank2Enable     - bank 2 enable
     *     ddrMemCtrlDacSource       - DAC data source
     *     ddrMemCtrlBank0Pack       - bank bank 0 packing mode
     *     ddrMemCtrlBank1Pack       - bank bank 1 packing mode
     * Rather that setting them individually, they will be set using the
     * ReadyFlow library function, P7142SetDdrMemCtrlParams().  Bank
     * start and bank depth (in 32-bit words) are also set.
     */
    switch (bank)
    {
        case P7142_DDR_MEM_BANK0:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_0_WRITE_MODE);
            ddrMemParams.ddrMemBank0StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank0Depth = /*P7142_DDR_MEM_BANK_MAX_DEPTH;*/(bankDepth/4);

                        ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth     = 0;

            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth     = 0;
        break;

        case P7142_DDR_MEM_BANK1:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_1_WRITE_MODE);
            ddrMemParams.ddrMemBank1StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank1Depth = /*P7142_DDR_MEM_BANK_MAX_DEPTH;*/(bankDepth/4);

                        ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth     = 0;

            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth     = 0;
        break;

        case P7142_DDR_MEM_BANK2:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_2_WRITE_MODE);
            ddrMemParams.ddrMemBank2StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank2Depth = /*P7142_DDR_MEM_BANK_MAX_DEPTH;*/(bankDepth/4);

                        ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth     = 0;

            ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth     = 0;
        break;
    }

    /* apply the parameter table to the registers */
    P7142InitDdrMemRegs (&ddrMemParams, &(p7142Regs->BAR2RegAddr));


    /* start the transfer, wait for completion --------------------------- */

    /* FIFO enable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                     \
        p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl, \
        P7142_FIFO_ENABLE);

    /* Sync Io Caches*/
    PTK714X_DMASyncCpu(&dmaBuf);

    /* write to DDR memory bank */
    P7142DmaStart(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_7);

    /* wait for interrupt completion */
    status = sem_wait(&memWriteDmaSem);
    if (status != 0)
        {
        sem_destroy(&memWriteDmaSem);
        PTK714X_DMAIntDisable(dmaWriteHandle);
        PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaWriteHandle);
        return (8);
        }

    /* verify that all data was written ---------------------------------- */

    /* get last address so we can verify that we wrote all data */
    P7142_GET_DDR_MEM_CAPTURE_END_ADDR(                                \
        p7142Regs->BAR2RegAddr.ddrMem.ddrMemBankCaptEndAddr[bank].Lsb, \
        lastAddr);

    /* check read operation */
    if (lastAddr != bankEndAddr)
    {
        /* determine how many extra bytes to write */
        overWriteVal = (bankEndAddr - lastAddr) << 3;

        /* set up channel parameters */
        P7142DmaChanSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                          PCI7142_DMA_CMD_STAT_DMA_ENABLE,
                          PCI7142_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                          PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
                          2048,            /* Max Burst Count */
                          0);

        /* setup descriptor parameters */
        P7142DmaDescptrSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                             P7142_DMA_DESCPTR_0,
                             overWriteVal, /* Transfer Interval Count */
                             PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                             PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                             (unsigned long)dmaBuf.kernBuf);

        /* apply the parameters to the DMA registers */
        P7142DmaChanInit(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                         &(p7142Regs->BAR0RegAddr),
                         P7142_DMA_CHAN_7);

        /* write over-write buffer to DDR Memory Bank */
        P7142DmaStart(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_7);

        /* wait for interrupt completion */
        status = sem_wait(&memWriteDmaSem);
        if (status != 0)
        {
            sem_destroy(&memWriteDmaSem);
            PTK714X_DMAIntDisable(dmaWriteHandle);
            PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
            PTK714X_DMAClose(hDev, dmaWriteHandle);
            return (8);
        }
    }


    /* clean up and exit ------------------------------------------------- */

    /* disable DDR memory */
    P7142_SET_DDR_MEM_MODE(p7142Regs->BAR2RegAddr.ddrMem.ddrMemCtrl,   \
                           P7142_DDR_MEM_DISABLE_MODE);

    /* FIFO disable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                                   \
        p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl,               \
        P7142_FIFO_DISABLE);

    /* clean up */
    sem_destroy(&memWriteDmaSem);
    PTK714X_DMAIntDisable(dmaWriteHandle);
    PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
    PTK714X_DMAClose(hDev, dmaWriteHandle);

    return (0);
}


