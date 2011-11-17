#include "p71xx.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>


using namespace Pentek;

/// This function is called by windriver each time a dma transfer is complete.
/// The user data (pData) that is delivered with the interrupt contains a
/// pointer to an instance of p71xx. The p71xx::dmaInterrupt() method is called
/// to handle the actual processing of the dma transfer.
///
/// DMA interrupts are cleared by the Kernel Device Driver.
///
/// DMA interrupts are enabled when this routine is executed.
/// @param hDev The 7142 Device Handle
/// @param dmaChannel - DMA channel generating the interrupt(0-3)
/// @param pData - Pointer to user defined data
/// @param pIntResults - Pointer to the interrupt results structure
void dmaIntHandler(
		PVOID hDev,
		unsigned int dmaChannel,
		PVOID pData,
        PTK714X_INT_RESULT *pIntResult)
{

	// Cast the user data to DmaHandlerData*
	DmaHandlerData* dmaData = (DmaHandlerData*) pData;

	// Call the dmaInterrupt member function in the p71xx object
	dmaData->p71xx->dmaInterrupt(dmaChannel);
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

    /* cleanup for exit */
    PTK714X_DeviceClose(_deviceHandle);
    PTK714X_LibUninit();
    std::cout << "ReadyFlow closed" << std::endl;

    return;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p71xx::dmaInterrupt(int chan) {
	/// @todo this logic needs to be refactored so that there is no dynamic allocation
	/// happening during dma interrrupts.
	std::vector<char> data;
	data.resize(_dmaBufSize);

	/// Copy the data into the vector
	memcpy(&data[0], (char*)dmaBuf[chan].usrBuf + _chainIndex[chan]*_dmaBufSize, _dmaBufSize);

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
p71xx::read(int chan, char* buf, int bytes) {

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

    /* Reset board registers to default values */
    P7142ResetRegs (&_p7142Regs);

    /* Load parameter tables with default values */
    P7142SetPciDefaults    (&_p7142PciParams);
    P7142SetDmaDefaults    (&_p7142DmaParams);
    P7142SetBoardDefaults  (&_p7142BoardParams);
    P7142SetInputDefaults  (&_p7142InParams);
    P7142SetOutputDefaults (&_p7142OutParams);
    P7142SetDac5687Defaults(&_p7142Dac5686Params);

    // Apply our adjustments
    configDmaParameters();
    configBoardParameters();
    configInParameters();
    configOutParameters();
    configDacParameters();

    /* Write parameter table values to the 7142 registers */
    P7142InitPciRegs    (&_p7142PciParams,   &(_p7142Regs.BAR0RegAddr));
    P7142InitDmaRegs    (&_p7142DmaParams,   &(_p7142Regs.BAR0RegAddr));
    P7142InitBoardRegs  (&_p7142BoardParams, &(_p7142Regs.BAR2RegAddr));
    P7142InitInputRegs  (&_p7142InParams,    &(_p7142Regs.BAR2RegAddr));
    P7142InitOutputRegs (&_p7142OutParams,   &(_p7142Regs.BAR2RegAddr));
    P7142InitDac5687Regs (&_p7142OutParams,   &_p7142Dac5686Params,   &(_p7142Regs.BAR2RegAddr));

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

	for (int chan = 0; chan < 4; chan++) {

		// open a DMA channel (required before we allocate the buffer)
		status = PTK714X_DMAOpen(_deviceHandle, chan, &dmaHandle[chan]);
		if (status != PTK714X_STATUS_OK) {
			std::cerr << __PRETTY_FUNCTION__ << ": Unable to open DMA channel " << chan << std::endl;
			abort();
		}
		// allocate DMA buffers, one per channel. All descriptors
		// for the channel will use consecutive areas in this buffer
		status = PTK714X_DMAAllocMem(dmaHandle[chan], _dmaBufSize*4, &dmaBuf[chan], (BOOL)0);
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
			(unsigned long)dmaBuf[chan].kernBuf+d*_dmaBufSize);    /* buffer address */
		}

		/* flush FIFO */
		P7142FlushFifo(&(_p7142Regs.BAR2RegAddr.adcFifo[chan]),
					   &(_p7142InParams.adcFifo[chan]));

		/* Flush the CPU caches */
		PTK714X_DMASyncCpu(&dmaBuf[chan]);

		// initialize the dma chain index
		_chainIndex[chan] = 0;

		// Initialize the data that will be delivered to the dma interrupt handler
		_dmaHandlerData[chan].chan  = chan;
		_dmaHandlerData[chan].p71xx = this;


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

    _p7142OutParams.dacPllVdd = P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE;
    /// @todo Is the following correct for PLL usage? What do they mean by "bypass"?
    _p7142OutParams.dacClkSel = P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS;
    _p7142OutParams.outputSyncBusSel = P7142_SYNC_BUS_SEL_A;
    _p7142OutParams.outputRefClkFreq = 48.0e6; /// @todo need to get the correct freq. brought into here.
    _p7142OutParams.dacFifo.fifoPackMode = P7142_FIFO_DAC_PACK_MODE_16BIT_PACK;

    // enable or disable word swap
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
void p71xx::configDacParameters() {

	/// @todo This will be used just for initialization. The DAC
	/// NCO will be reprogrammed by p7142Up to match the required IF
	_p7142Dac5686Params.ncoFrequency = 48000000.0;

    _p7142Dac5686Params.bypassMode = P7142_DAC_FULL_BYPASS_DISABLE;

    // Calculate parameters for PLL operation, and check for errors.
    // Actually, the DAC registers related to this will be reprogrammed
    // by p7142Up.
	int temp = DAC5687DacPllClkGenSetup(
			   _p7142OutParams.outputRefClkFreq,
			   _p7142Dac5686Params.ncoFrequency,
			   &(_p7142Dac5686Params.pllDividerRatio),
			   &(_p7142Dac5686Params.pllFreq),
			   &(_p7142Dac5686Params.pllKv),
			   &(_p7142Dac5686Params.filterSelect));
	if (temp) {
		std::cerr
		<< __FILE__
				<< ":" << __FUNCTION__
				<< " - DAC5687DacPllClkGenSetup was not happy trying to compute the DAC configuration"
				<< std::endl;
	}

    _p7142Dac5686Params.dacACourseGain = 0xf;
    _p7142Dac5686Params.dacAFineGain   = 0x00;
    _p7142Dac5686Params.dacBCourseGain = 0xf;
    _p7142Dac5686Params.dacBFineGain   = 0x00;
    _p7142Dac5686Params.inputMode      = DAC5687_CFG1_DATA_IN_TWOS_COMPLIMENT;

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
	int status = PTK714X_DMAIntEnable(dmaHandle[chan],
								   PTK714X_DMA_DESCRIPTOR_FINISH,
								   &_dmaHandlerData[chan],
								   (PTK714X_INT_HANDLER)dmaIntHandler);

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
   status = PTK714X_DMAIntDisable(dmaHandle[chan]);
	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ <<
				": DMA interrupt disable failed" << std::endl;
	}

	status = PTK714X_DMAFreeMem(dmaHandle[chan], &dmaBuf[chan]);
	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ <<
		   ": DMA memory free failed" << std::endl;
	}

	status = PTK714X_DMAClose(_deviceHandle, dmaHandle[chan]);
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
