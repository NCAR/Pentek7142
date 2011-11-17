#include "p7142.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>

using namespace Pentek;
//int drv_peekL(int fd,unsigned intaddr);
//int drv_pokeL(int fd,unsigned int addr,unsigned int value);

/* Global references */
/// @todo This will need to be fixed when we try to access multiple Pentek cards
sem_t dmaWriteSemHandle;
/// @todo This will need to be fixed when we try to access multiple Pentek cards
/************************************************************************
 Function: dmaWriteIntHandler

 Description: This routine is an interrupt handler for the DMA Descriptor
              Finish interrupt for DMA Channel 7.

 Inputs:      hDev        - 7142 Device Handle
              lintSource  - Local interrupt source generating the
                            interrupt(0-15).  In this example, this
                            corresponds with dmaChannel.
              pIntResults - Pointer to the interrupt results structure

 Returns:     none

 Notes:       Fifo interrupts are cleared by the Kernel Device Driver.
              Fifo interrupts are enabled when this routine is executed.
************************************************************************/
void dmaWriteIntHandler(PVOID               hDev,
                        int                lintSource,
                        PVOID               pData,
                        PTK714X_INT_RESULT *pIntResult)
{
    sem_post(&dmaWriteSemHandle);
}


////////////////////////////////////////////////////////////////////////////////
p7142::p7142(std::string devName, int dmaBufferSize, bool simulate):
  p71xx(devName, dmaBufferSize, simulate), _downconverters(P7142_NCHANNELS), _upconverter(0)
{
}

////////////////////////////////////////////////////////////////////////////////
p7142::~p7142() {
    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);
    for (int i = 0; i < P7142_NCHANNELS; i++) {
        delete _downconverters[i];
    }
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn*
p7142::addDownconverter(int chanId, int bypassdivrate,
        int simWavelength, bool sim4bytes) {
    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);
    // Just construct a new downconverter and put it in our list.
    p7142Dn* downconverter = new p7142Dn(
    		this,
    		chanId,
    		bypassdivrate,
            simWavelength,
            sim4bytes);
    addDownconverter(downconverter);
    return(downconverter);
}

////////////////////////////////////////////////////////////////////////////////
p7142Up*
p7142::addUpconverter(
		double sampleClockHz,
        double ncoFreqHz,
        char mode) {
    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);
    // Just construct a new upconverter and put it in our list.
    p7142Up* upconverter = new p7142Up(
    		this,
    		sampleClockHz,
    		ncoFreqHz,
    		mode);
    _addUpconverter(upconverter);
    return(upconverter);
}

////////////////////////////////////////////////////////////////////////////////
void
p7142::addDownconverter(p7142Dn * downconverter) {
    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);
    
    int chanId = downconverter->chanId();
    if (_downconverters[chanId]) {
        std::cerr << "Existing downconverter for channel " << chanId <<
                " is being replaced on device " << _devName << std::endl;
        delete _downconverters[chanId];
    }
    _downconverters[chanId] = downconverter;
}

////////////////////////////////////////////////////////////////////////////////
void
p7142::_addUpconverter(p7142Up * upconverter) {
    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);

    if (_upconverter) {
        std::cerr << "Existing upconverter is being replaced on device " << 
                _devName << std::endl;
        delete _upconverter;
    }
    _upconverter = upconverter;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142::resetDCM() {
    boost::recursive_mutex::scoped_lock guard(_p71xxMutex);

    if (isSimulating())
        return;

    // cycle the digital clock manager
    // hold the dcm in reset for a short period
    P7142_SET_DCM_CTRL_DCM_RST(_p7142Regs.BAR2RegAddr.dcmControl, P7142_DCM_CTRL_DCM_RST_RESET);
    usleep(1000);
    // take the dcm out of reset
    P7142_SET_DCM_CTRL_DCM_RST(_p7142Regs.BAR2RegAddr.dcmControl, P7142_DCM_CTRL_DCM_RST_RUN);
    usleep(1000);

    std::cout << "DCM has been cycled." << std::endl;

    return;

}

////////////////////////////////////////////////////////////////////////////////////////
int p7142::ddrMemWrite (P7142_REG_ADDR* p7142Regs,
                 unsigned int    bank,
                 unsigned int    bankStartAddr,
                 unsigned int    bankDepth,
                 int32_t*        dataBuf,
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
                                  NULL, (PTK714X_INT_HANDLER)dmaWriteIntHandler);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, dmaWriteHandle);
        return (6);
    }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&dmaWriteSemHandle,0,0))<0)
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
            ddrMemParams.ddrMemBank0Depth = (bankDepth/4);
        break;

        case P7142_DDR_MEM_BANK1:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_1_WRITE_MODE);
            ddrMemParams.ddrMemBank1StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank1Depth = (bankDepth/4);
        break;

        case P7142_DDR_MEM_BANK2:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_2_WRITE_MODE);
            ddrMemParams.ddrMemBank2StartAddr = bankStartAddr;
            ddrMemParams.ddrMemBank2Depth = (bankDepth/4);
        break;
    }

    /* apply the parameter table to the registers */
    P7142InitDdrMemRegs (&ddrMemParams, &(p7142Regs->BAR2RegAddr));


    /* start the transfer, wait for completion --------------------------- */

    /* FIFO enable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                     \
        p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl, \
        P7142_FIFO_ENABLE);

    /* Sync Cpu Caches*/
    PTK714X_DMASyncCpu(&dmaBuf);

    /* write to DDR memory bank */
    P7142DmaStart(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_7);

    /* wait for interrupt completion */
    status = sem_wait(&dmaWriteSemHandle);
    if (status != 0)
    {
        sem_destroy(&dmaWriteSemHandle);
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
        status = sem_wait(&dmaWriteSemHandle);
        if (status != 0)
        {
            sem_destroy(&dmaWriteSemHandle);
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
    sem_destroy(&dmaWriteSemHandle);
    PTK714X_DMAIntDisable(dmaWriteHandle);
    PTK714X_DMAFreeMem(dmaWriteHandle, &dmaBuf);
    PTK714X_DMAClose(hDev, dmaWriteHandle);

    return (0);
}
