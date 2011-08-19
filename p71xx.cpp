#include "p71xx.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>


using namespace Pentek;

sem_t dmaSemKey;

/************************************************************************
 Function: dmaIntHandler

 Description: This routine serves as the User Mode interrupt handler
              for this example.

 Inputs:      hDev - 7142 Device Handle
              dmaChannel - DMA channel generating the interrupt(0-3)
              pData - Pointer to user defined data
              pIntResults - Pointer to the interrupt results structure

 Returns:     none

 Notes:       DMA interrupts are cleared by the Kernel Device Driver.
              DMA interrupts are enabled when this routine is executed.
************************************************************************/
extern "C" void dmaIntHandler(
		PVOID hDev,
		unsigned int dmaChannel,
		PVOID pData,
        PTK714X_INT_RESULT *pIntResult);

int dmaCount[4] = {0,0,0,0};
long dmaTotal = 0;

void dmaIntHandler(
		PVOID hDev,
		unsigned int dmaChannel,
		PVOID pData,
        PTK714X_INT_RESULT *pIntResult)
{
	dmaTotal++;
	dmaCount[dmaChannel]++;
	if (!(dmaTotal % 500)) {
		for (int i = 0; i < 4; i++) {
			printf("%d ", dmaCount[i]);
		}
		printf("\n");
	}

    sem_post(&dmaSemKey);
}

////////////////////////////////////////////////////////////////////////////////////////
p71xx::p71xx(std::string devName, bool simulate):
_devName(devName),
_ctrlFd(-1),
_simulate(simulate),
_mutex(),
_isReady(false)
{
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // If we're simulating, things are simple...
	if (_simulate) {
		_isReady = true;
		return;
	}

	for (int adchan = 0; adchan < 4; adchan++) {
		_adcActive[adchan] = false;
	}
	// initialize ReadyFlow
	_isReady = initReadyFlow();

	return;
}

////////////////////////////////////////////////////////////////////////////////////////
p71xx::~p71xx() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    /* cleanup for exit */
    sem_destroy(&dmaSemKey);
    PTK714X_DeviceClose(hDev);
    PTK714X_LibUninit();
    std::cout << "ReadyFlow closed" << std::endl;

    return;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p71xx::initReadyFlow() {
	slot = -1;
    hDev = NULL;

    /* initialize the PTK714X library */
    DWORD dwStatus = PTK714X_LibInit();
    if (dwStatus != PTK714X_STATUS_OK)
    {
        std::cerr << "Failed to initialize the PTK714X library" << std::endl;
        return false;
    }

    /* Find and open a PTK714X device (by default ID) */
    hDev = PTK714X_DeviceFindAndOpen(&slot, &BAR0Base, &BAR2Base);
    if (hDev == NULL)
    {
        std::cerr << "Pentek 7142 device not found" << std::endl;
    }

    /* Initialize 7142 register address tables */
    P7142InitRegAddr (BAR0Base, BAR2Base, &p7142Regs);

    /* check if module is a 7142 */
    P7142_GET_MODULE_ID(p7142Regs.BAR2RegAddr.idReadout, moduleId);
    if (moduleId != P7142_MODULE_ID)
    {
        std::cerr << "Failed to identify a Pentek 7142 module." << std::endl;
        return false;
    }

    std::cout << "Pentek 7142 device";
    std::cout << std::hex << " BAR0: 0x" << (void *)BAR0Base;
    std::cout << std::hex << " BAR2: 0x" << (void *)BAR2Base;
    std::cout << std::dec;
    std::cout <<std::endl;

    /* Reset board registers to default values */
    P7142ResetRegs (&p7142Regs);

    /* Load parameter tables with default values */
    P7142SetPciDefaults    (&p7142PciParams);
    P7142SetDmaDefaults    (&p7142DmaParams);
    P7142SetBoardDefaults  (&p7142BoardParams);
    P7142SetInputDefaults  (&p7142InParams);
    P7142SetOutputDefaults (&p7142OutParams);
    P7142SetDac5687Defaults(&p7142Dac5686Params);

    // Apply our adjustments
    configDmaParameters();
    configBoardParameters();
    configInParameters();
    configOutParameters();
    configDacParameters();

    /* Write parameter table values to the 7142 registers */
    P7142InitPciRegs    (&p7142PciParams,   &(p7142Regs.BAR0RegAddr));
    P7142InitDmaRegs    (&p7142DmaParams,   &(p7142Regs.BAR0RegAddr));
    P7142InitBoardRegs  (&p7142BoardParams, &(p7142Regs.BAR2RegAddr));
    P7142InitInputRegs  (&p7142InParams,    &(p7142Regs.BAR2RegAddr));
    P7142InitOutputRegs (&p7142OutParams,   &(p7142Regs.BAR2RegAddr));
    P7142InitDac5687Regs (&p7142OutParams,   &p7142Dac5686Params,   &(p7142Regs.BAR2RegAddr));

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configBoardParameters() {

    // Board customization

    p7142BoardParams.busAMaster      = P7142_MSTR_CTRL_MASTER;
    p7142BoardParams.busATermination = P7142_MSTR_CTRL_TERMINATED;

    p7142BoardParams.busASelectClock = P7142_MSTR_CTRL_SEL_CLK_OSCILLATOR;
    p7142BoardParams.busAClockSource = P7142_MSTR_CTRL_CLK_SRC_LVDS_BUS;

    p7142BoardParams.busASelectSync  = P7142_MSTR_CTRL_SEL_SYNC_REGISTER;
    p7142BoardParams.busASyncSource  = P7142_MSTR_CTRL_SYNC_SRC_LVDS_BUS;

    p7142BoardParams.busASelectGate  = P7142_MSTR_CTRL_SEL_GATE_REGISTER;
    p7142BoardParams.busAGateSource  = P7142_MSTR_CTRL_GATE_SRC_LVDS_BUS;

    /* Bus B parameters */

    p7142BoardParams.busBMaster      = P7142_MSTR_CTRL_MASTER;
    p7142BoardParams.busBTermination = P7142_MSTR_CTRL_TERMINATED;

    p7142BoardParams.busBSelectClock = P7142_MSTR_CTRL_SEL_CLK_OSCILLATOR;
    p7142BoardParams.busBClockSource = P7142_MSTR_CTRL_CLK_SRC_LVDS_BUS;

    p7142BoardParams.busBSelectSync  = P7142_MSTR_CTRL_SEL_SYNC_REGISTER;
    p7142BoardParams.busBSyncSource  = P7142_MSTR_CTRL_SYNC_SRC_LVDS_BUS;

    p7142BoardParams.busBSelectGate  = P7142_MSTR_CTRL_SEL_GATE_REGISTER;
    p7142BoardParams.busBGateSource  = P7142_MSTR_CTRL_GATE_SRC_LVDS_BUS;

    p7142BoardParams.endianness = P7142_MISC_CTRL_ENDIANNESS_LE;

}
////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configDmaParameters() {

    // Board customization

    /* set up DMA parameters - only non-default values are shown
     *
     * Set parameters for the selected DMA channel.  The program uses only
     * DMA descriptor and transfer size is set to buffer size.
     */

	/* Create a DMA Complete semaphore */
	if((sem_init(&dmaSemKey,0,0))<0) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ": Unable to create a DMA complete semaphore." << std::endl;
		abort();
	}

    int dmaBufSize = 65536;
	int status;

	for (int chan = 0; chan < 4; chan++) {
		// open a DMA channel (required before we allocate the buffer)
		status = PTK714X_DMAOpen(hDev, chan, &dmaHandle[chan]);
		if (status != PTK714X_STATUS_OK) {
			std::cerr << __FILE__ << ":" << __FUNCTION__ << ": Unable to open DMA channel " << chan << std::endl;
			abort();
		}
		std::cout << "DMA opened for channel " << chan << ", handle is " << dmaHandle[chan] << std::endl;
		// allocate DMA buffers, one per channel. All descriptors
		// for the channel will use consecutive areas in this buffer
		status = PTK714X_DMAAllocMem(dmaHandle[chan], dmaBufSize*4, &dmaBuf[chan], (BOOL)0);
		if (status != PTK714X_STATUS_OK) {
			std::cerr << __FILE__ << ":" << __FUNCTION__ << ": Unable to allocate a DMA buffer for channel " << chan << std::endl;
			abort();
		}
		std::cout << "DMA buffer allocated for channel " << chan << ", buffer is " << std::hex << dmaBuf[chan].usrBuf << std::endl;

		/* Abort any existing transfers */
		P7142DmaAbort(&(p7142Regs.BAR0RegAddr), chan);

		/* Flush DMA channel buffer */
		P7142DmaFlush(&(p7142Regs.BAR0RegAddr), chan);

		// set up channel parameters */
		P7142DmaChanSetup(&(p7142DmaParams.dmaChan[chan]),
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
			&(p7142DmaParams.dmaChan[chan]),
			d,                                                     /* descriptor number */
			dmaBufSize,                                            /* transfer count in bytes */
			PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,             /* DMA interrupt */
			PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_NEXT,               /* type of descriptor */
			(unsigned long)dmaBuf[chan].kernBuf+d*dmaBufSize);     /* buffer address */
		}

		/* flush FIFO */
		P7142FlushFifo(&(p7142Regs.BAR2RegAddr.adcFifo[chan]),
					   &(p7142InParams.adcFifo[chan]));

		/* Flush the CPU caches */
		PTK714X_DMASyncCpu(&dmaBuf[chan]);
	}

}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configInParameters() {

    /* data input parameters - the program uses Sync Bus A for clock, sync
     * and gate signals.  This is the ReadyFlow default but is provided for
     * user experimentation.  Besides that, only parameters for the FIFO
     * for the active channel is set in this example.
     */

    /* Sync Bus select */
    p7142InParams.inputSyncBusSel = P7142_SYNC_BUS_SEL_A;

	///@todo Temporarily using the GateFlow gating signal to
	/// enable/disable data flow. This will be changed to
	/// using the timers for this purpose.
    ///
    /* set the gate generator register pointers */
    if (p7142InParams.inputSyncBusSel == P7142_SYNC_BUS_SEL_A)
        gateGenReg = (volatile unsigned int *)(p7142Regs.BAR2RegAddr.gateAGen);
    else
        gateGenReg = (volatile unsigned int *)(p7142Regs.BAR2RegAddr.gateBGen);
    /* disable FIFO writes (set Gate in reset) */
	P7142_SET_GATE_GEN(&gateGenReg, P7142_GATE_GEN_DISABLE);

    /* set FIFO parameters */

    for (int adchan = 0; adchan < 1; adchan++) {
    	/* select data packing mode.  It can be unpacked or time-packed.
		 * The program define is located at the top of the program.
		 */
		p7142InParams.adcFifo[adchan].fifoPackMode = P7142_FIFO_ADC_PACK_MODE_TIME_PACK;

		/* set FIFO decimation.  This allows the input data rate to the
		 * FIFO to be reduced.  It can be a value from 0 to 0xFFF.  Actual
		 * decimation is this value plus one.
		 */
		p7142InParams.adcFifoDecimation[adchan] = 7;

		/* The FIFO Almost Full and Almost Empty levels are set to default
		 * values for all programs.  The values shown here are the default
		 * values and and are provided to show usage.  Their values must be
		 * chosen to work with the DMA channel maximum burst count value.
		 */
		p7142InParams.adcFifo[adchan].fifoAlmostEmptyLevel = 512;
		p7142InParams.adcFifo[adchan].fifoAlmostFullLevel  = 544;
    }


}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configOutParameters() {

    // DAC customization
    p7142OutParams.dacPllVdd = P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE;
    /// @todo is the following correct for PLL usage? What do they mean by "bypass"?
    p7142OutParams.dacClkSel = P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS;
    p7142OutParams.outputSyncBusSel = P7142_SYNC_BUS_SEL_A;
    p7142OutParams.outputRefClkFreq = 48.0e6; /// @todo need to get the correct freq. brought into here.
    p7142OutParams.dacFifo.fifoPackMode = P7142_FIFO_DAC_PACK_MODE_16BIT_PACK;

    /* enable or disable word swap */
    p7142OutParams.dacFifo.fifoWordSwap = P7142_FIFO_CTRL_WORD_SWAP_DISABLE;

    /* The FIFO Almost Full and Almost Empty levels are set to default
     * values for all programs.  The values shown here are the default
     * values for DAC FIFOs and are provided to show usage.   Their values
     * must be chosen to work with the DMA channel maximum burst count
     * value.
     */
    p7142OutParams.dacFifo.fifoAlmostEmptyLevel = 6144;
    p7142OutParams.dacFifo.fifoAlmostFullLevel  = 6176;
}

////////////////////////////////////////////////////////////////////////////////////////
void p71xx::configDacParameters() {

	/// @todo This will be used just for initialization. The DAC
	/// NCO will be reprogrammed by p7142Up to match the IF
	p7142Dac5686Params.ncoFrequency = 48000000.0;

    p7142Dac5686Params.bypassMode = P7142_DAC_FULL_BYPASS_DISABLE;
    /// Calculate parameters for PLL operation, check for errors.
    /// Actually, the DAC registers related to this will be programmed
    // by p7142Up.
	int temp = DAC5687DacPllClkGenSetup(
			   p7142OutParams.outputRefClkFreq,
			   p7142Dac5686Params.ncoFrequency,
			   &(p7142Dac5686Params.pllDividerRatio),
			   &(p7142Dac5686Params.pllFreq),
			   &(p7142Dac5686Params.pllKv),
			   &(p7142Dac5686Params.filterSelect));
	if (temp) {
		std::cerr
		<< __FILE__
				<< ":" << __FUNCTION__
				<< " - DAC5687DacPllClkGenSetup was not happy trying to compute the DAC configuration"
				<< std::endl;
	}

    p7142Dac5686Params.dacACourseGain = 0xf;
    p7142Dac5686Params.dacAFineGain   = 0x00;
    p7142Dac5686Params.dacBCourseGain = 0xf;
    p7142Dac5686Params.dacBFineGain   = 0x00;
    p7142Dac5686Params.inputMode      = DAC5687_CFG1_DATA_IN_TWOS_COMPLIMENT;

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
	P7142DmaChanInit(&(p7142DmaParams.dmaChan[chan]),
					 &(p7142Regs.BAR0RegAddr),
					 chan);

	// enable the FIFO
	P7142_SET_FIFO_CTRL_FIFO_ENABLE(                            \
		p7142Regs.BAR2RegAddr.adcFifo[chan].FifoCtrl, \
		P7142_FIFO_ENABLE);

	// enable the DMA interrupt, on descriptor finish
	int status = PTK714X_DMAIntEnable(dmaHandle[chan],
								   PTK714X_DMA_DESCRIPTOR_FINISH,
								   NULL,
								   (PTK714X_INT_HANDLER)dmaIntHandler);

	if (status != PTK714X_STATUS_OK) {
		std::cerr << __FILE__ << ":" << __FUNCTION__ << ": Unable to enable DMA interrupt for channel " << chan << std::endl;
		abort();
	}

	std::cout << "DMA interrupt enabled for channel " << chan << std::endl;

	/* start DMA */
	P7142DmaStart(&(p7142Regs.BAR0RegAddr), chan);

	_adcActive[chan] = true;

	/* When DMA has completed, it will post the semphore */
	//int status = sem_wait(&dmaSemKey);
	//if (status != 0)
	//	printf("Timeout waiting for DMA complete semaphore for DMA %d\n", chan);

	//std::cout << "semaphore received" << std::endl;

}

////////////////////////////////////////////////////////////////////////////////////////
void
p71xx::start() {
	/* enable FIFO writes (release Gate from reset) */
	///@todo Temporarily using the GateFlow gating signal to
	/// enable/disable data flow. This will be changed to
	/// using the timers for this purpose.
	P7142_SET_GATE_GEN(&gateGenReg, P7142_GATE_GEN_ENABLE);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p71xx::stop() {

    /* disable FIFO writes (set Gate in reset) */
	///@todo Temoprarily using the GateFlow gating signal to
	/// enable/disable data flow. This will be changed to
	/// using the timers for this purpose.
	P7142_SET_GATE_GEN(&gateGenReg, P7142_GATE_GEN_DISABLE);

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
    P7142DmaAbort(&(p7142Regs.BAR0RegAddr), chan);

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

	status = PTK714X_DMAClose(hDev, dmaHandle[chan]);
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
	BUFFER_CFG bc;
	bc.bufno = 0;
	bc.bufsize = bufN*intbufsize;
	bc.intbufsize = intbufsize;
	bc.physAddr = 0;

	int status = ioctl(fd, BUFSET, &bc);
	if (status == -1) {
		std::cout << "Error setting pentek buffer sizes" << std::endl;
		perror("");
	}
	return status;
}
