#include "p71xx.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>


using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p71xx::p71xx(std::string devName, bool simulate):
_ok(false),
_devName(devName),
_ctrlFd(-1),
_simulate(simulate),
_mutex()
{
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // If we're simulating, things are simple...
	if (_simulate) {
		_ok = true;
		return;
	}

	// initialize ReadyFlow
	_ok = initReadyFlow();

	return;

	_ok = true;
}

////////////////////////////////////////////////////////////////////////////////////////
p71xx::~p71xx() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    /* cleanup for exit */
    PTK714X_DeviceClose(hDev);
    PTK714X_LibUninit();
    std::cout << "ReadyFlow closed" << std::endl;

    return;

    if (_ctrlFd >= 0)
        close(_ctrlFd);
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p71xx::initReadyFlow() {
	slot = -1;
    hDev         = NULL;
	intrStat     = PTK714X_STATUS_OK;
	dmaBufStat   = PTK714X_STATUS_OK;
	dmaHandle    = NULL;
	adcData      = NULL;

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
    P7142SetPciDefaults   (&p7142PciParams);
    P7142SetDmaDefaults   (&p7142DmaParams);
    P7142SetBoardDefaults (&p7142BoardParams);
    P7142SetInputDefaults (&p7142InParams);
    P7142SetOutputDefaults(&p7142OutParams);

    // Board customization
    p7142BoardParams.busASelectClock = P7142_MSTR_CTRL_SEL_CLK_EXT_CLK;
    p7142BoardParams.busAClockSource = P7142_MSTR_CTRL_CLK_SRC_SEL_CLK;

    // DAC customization
    p7142OutParams.dacPllVdd = P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE;
    p7142OutParams.dacClkSel = P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS;

    /* Apply parameter table values to the 7142 registers */
    P7142InitPciRegs    (&p7142PciParams,   &(p7142Regs.BAR0RegAddr));
    P7142InitDmaRegs    (&p7142DmaParams,   &(p7142Regs.BAR0RegAddr));
    P7142InitBoardRegs  (&p7142BoardParams, &(p7142Regs.BAR2RegAddr));
    P7142InitInputRegs  (&p7142InParams,    &(p7142Regs.BAR2RegAddr));
    P7142InitOutputRegs (&p7142OutParams,   &(p7142Regs.BAR2RegAddr));

    // set the pointers for the sd3c control registers
    svnRevReg   = (DWORD*)(BAR2Base + 0x8000 + 8*481);
    tcvrCtrlReg = (DWORD*)(BAR2Base + 0x8000 + 8*463);

    /// @todo Enable free run for now. Will need to remove this later
    *tcvrCtrlReg = 1;

    std::cout
    	<< "sd3c revision: "
    	<< (*svnRevReg & 0x3fff)
    	<< "  ddc type: "
    	<< ((*svnRevReg >> 14) & 0x3)
    	<< std::endl;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p71xx::ok() const {
	return _ok;
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

//////////////////////////////////////////////////////////////////////
int
p71xx::doIoctl(int fd, int ioctlCode, void* arg, std::string errMsg, bool doexit) {
   int status = ioctl(fd, ioctlCode, arg);
   if (status == -1) {
    std::cout << errMsg << std::endl;
    perror("");
    if (doexit)
    exit(1);
   }

   return status;
}

//////////////////////////////////////////////////////////////////////
int
p71xx::doIoctl(int fd, int ioctlCode, int arg, std::string errMsg, bool doexit) {
   return doIoctl(fd, ioctlCode, (void*) arg, errMsg, doexit);
}

//////////////////////////////////////////////////////////////////////
int
p71xx::doIoctl(int fd, int ioctlCode, double arg, std::string errMsg, bool doexit) {

   double doubleArg = arg;
   return doIoctl(fd, ioctlCode, (void*) &doubleArg, errMsg, doexit);
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
