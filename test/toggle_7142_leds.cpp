#include <iomanip>
#include <iostream>
#include <string>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <sched.h>
#include <sys/timeb.h>
#include <ctime>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>
#include <logx/Logging.h>
#include <toolsa/pmu.h>

#include "p7142sd3c.h"
#include "p7142Up.h"

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

LOGGING("toggleP7142LEDs")

int _nToggles = 5; ///< number of times leds are toggled
double _waitSecs = 0.5;  ///< Wait between ids (secs)

//////////////////////////////////////////////////////////////////////
//
/// Parse the command line options, and also set some options
/// that are not specified on the command line.
/// @return The runtime options that can be passed to the
/// threads that interact with the RR314.
void parseOptions(int argc, char** argv)
{
  
  // get the options
  po::options_description descripts("Options");
  descripts.add_options()
    ("help", "Describe options")
    ("nToggles", po::value<int>(&_nToggles),
     "No. of times lights are toggled (default 5)")
    ("waitSecs", po::value<double>(&_waitSecs), 
     "Wait between toggles (secs) (default 0.5)")
    ;

  po::variables_map vm;
  po::command_line_parser parser(argc, argv);
  po::positional_options_description pd;
  po::store(parser.options(descripts).positional(pd).run(), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    cout << "Usage: " << argv[0] << 
      " [OPTION]..." << endl;
    cout << descripts << endl;
    exit(0);
  }
  
}

///////////////////////////////////////////////////////////
/// @return The current time, in seconds since Jan. 1, 1970.
double nowTime()
{
  struct timeb timeB;
  ftime(&timeB);
  return timeB.time + timeB.millitm/1000.0;
}

/**************************************************************************
 Function: exitHandler
 
 Description:  Routine to set LEDs and output an appropriate message at
               program exit.  Provided to make example code cleaner.

 Inputs:       code - exit code
               bufA - pointer to data buffer A
               bufB - pointer to data buffer B
               bufC - pointer to data buffer C
 Return:       exit code 
**************************************************************************/

int exitHandler(int code, void *hDev)

{

  /* Perform necessary cleanup before exiting the program */
  if (hDev) {
    PTK714X_DeviceClose(hDev);
  }
  
  DWORD dwStatus = PTK714X_LibUninit();
  if (PTK714X_STATUS_OK != dwStatus) {
    puts ("Error: PTK7142 library un-init failed");
  }
    
  /* display message */
  switch (code) {
    case  0: puts ("Programming done");                          break;
    case  1: puts ("Error: input MCS file failed to open");      break;
    case  2: puts ("Error: PTK7142 library init failed");        break;
    case  3: puts ("Error: 7142 device not found");              break;
    case  4: puts ("Error: another initialization in progress"); break;
    case  5: puts ("Error: load did not complete");              break;
    case  6: puts ("Error: errors during configuration");        break;
  }
  
  return (code);

}

///////////////////////////////////////////////////////////
int
main(int argc, char** argv)
{

  // Let logx get and strip out its arguments
  logx::ParseLogArgs(argc, argv);
  
  // parse the command line options, substituting for config params.
  parseOptions(argc, argv);

  // set to ignore SIGPIPE errors which occur when sockets
  // are broken between client and server
  signal(SIGPIPE, SIG_IGN);
  
  // volatile unsigned int  cntr;
  // volatile unsigned int  dataVal;
  // unsigned int           imageSize;
  // volatile unsigned int  regVal;
  // volatile unsigned int  retVal;
  // volatile unsigned int  *virtexReg;
  
  /* Initialize the library */

  DWORD dwStatus = PTK714X_LibInit();
  if (PTK714X_STATUS_OK != dwStatus) {
    return (exitHandler (2, NULL));
  }
  
  // Find and open the next PTK714X device
  // user will be asked to pick the device num
  
  DWORD BAR0Base;
  DWORD BAR2Base;
  DWORD slot = -1; // forces user interaction if more than 1 pentek
  void *hDev = PTK714X_DeviceFindAndOpen(&slot, &BAR0Base, &BAR2Base);
  if (hDev == NULL) {
    return (exitHandler (3, hDev));
  }
  
  /* Initialize the 7142 register address table */
  P7142_REG_ADDR p7142Regs;  /* 7142 register table */
  P7142InitRegAddr(BAR0Base, BAR2Base, &p7142Regs);

  P7142_BAR0_REG_ADDR bar0 = p7142Regs.BAR0RegAddr;
  P7142_BAR2_REG_ADDR bar2 = p7142Regs.BAR2RegAddr;
  
  /* check if module is a 7142 */
  unsigned int moduleId;
  P7142_GET_MODULE_ID(p7142Regs.BAR2RegAddr.idReadout, moduleId);
  if (moduleId != P7142_MODULE_ID) {
    cerr << "Pentek card " << slot + 1 << " is not a 7142!" <<
      endl;
    cerr << "Expected 0x" << hex << P7142_MODULE_ID << 
      ", and got 0x" << moduleId << dec << endl;
    return false;
  }

  // print status

  cerr << "Pentek 7142 device";
  cerr << "  device: " << hex << hDev << endl;
  cerr << "  slot: " << dec << slot << endl;
  cerr << "  BAR0Base: " << hex << (void *)BAR0Base;
  cerr << "  BAR2Base: " << hex << (void *)BAR2Base;
  cerr << dec;
  cerr << endl;

  // get master bus controls

  uint32_t masterBusAControl;
  P7142_REG_READ(BAR2Base + P7142_MASTER_BUS_A_CONTROL, masterBusAControl);
  
  uint32_t masterBusBControl;
  P7142_REG_READ(BAR2Base + P7142_MASTER_BUS_B_CONTROL, masterBusBControl);

  cerr << "Initial state:" << endl;
  cerr << "  masterBusAControl: " << hex << masterBusAControl << endl;
  cerr << "  masterBusBControl: " << hex << masterBusBControl << endl;
  cerr << dec;

  // toggle master control on and off, to toggle LEDs

  uint32_t onA = masterBusAControl & 0xfffffffe;
  uint32_t onB = masterBusBControl & 0xfffffffe;

  uint32_t offA = masterBusAControl | 0x00000001;
  uint32_t offB = masterBusBControl | 0x00000001;

  useconds_t sleepTime = (int) (_waitSecs * 1.0e6);

  for (int ii = 0; ii < _nToggles; ii++) {

    cerr << "--->> Toggling bus master on" << endl;
    
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, onA);
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, onB);

    usleep(sleepTime);

    cerr << "--->> Toggling bus master off" << endl;
    
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, offA);
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, offB);

    usleep(sleepTime);

  }

  // reset to the initial state
    
  cerr << "--->> Resetting to initial state" << endl;
    
  P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, masterBusAControl);
  P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, masterBusBControl);

  // return

  return (exitHandler (0, hDev));
  
}

#ifdef JUNK_FOR_NOW

  cerr << "0000000 pciIntrFlag: " << *bar0.pciIntrFlag << endl;
  for (int ii = 0; ii < 4; ii++) {
    cerr << "0000000 pciIntrEnable[" << ii << "]: " << *bar0.pciIntrEnable[ii] << endl;
  }
  cerr << "0000000 fpgaDataIn: " << *bar0.fpgaDataIn << endl;
  cerr << "0000000 fpagDataOut: " << *bar0.fpagDataOut << endl;
  cerr << "0000000 bdChanReset: " << *bar0.bdChanReset << endl;
  cerr << "0000000 fpgaRevision: " << *bar0.fpgaRevision << endl;
  cerr << "0000000 virtexConfig: " << *bar0.virtexConfig << endl;
  cerr << "0000000 pciDcmControl: " << *bar0.pciDcmControl << endl;
  cerr << "0000000 localDmaReqStatus: " << *bar0.localDmaReqStatus << endl;
  cerr << "0000000 pciBusStatus: " << *bar0.pciBusStatus << endl;
  cerr << "0000000 dmaPciIntrEna: " << *bar0.dmaPciIntrEna << endl;
  cerr << "0000000 lclDmaInRemap: " << *bar0.lclDmaInRemap << endl;
  cerr << "0000000 lclDmaOutRemap: " << *bar0.lclDmaOutRemap << endl;
  cerr << "0000000 dmaCommand: " << *bar0.dmaCommand << endl;
  for (int ii = 0; ii < 9; ii++) {
    cerr << "0000000 dmaCurrXferCounter[" << ii << "]: " << *bar0.dmaCurrXferCounter[ii] << endl;
    cerr << "0000000 dmaCurrPciAddr[" << ii << "]: " << *bar0.dmaCurrPciAddr[ii] << endl;
    cerr << "0000000 dmaCommandStatus[" << ii << "]: " << *bar0.dmaCommandStatus[ii] << endl;
    cerr << "0000000 dmaXferIntervalCounter[" << ii << "]: " << *bar0.dmaXferIntervalCounter[ii] << endl;
  }
  
  cerr << "222222 idReadout: " << *bar2.idReadout << endl;
  cerr << "222222 idReadout: " << *bar2.idReadout << endl;
  cerr << "222222 twsiPort: " << *bar2.twsiPort << endl;
  cerr << "222222 dcmControl: " << *bar2.dcmControl << endl;
  cerr << "222222 miscControl: " << *bar2.miscControl << endl;
  cerr << "222222 fpgaRevision1: " << *bar2.fpgaRevision1 << endl;
  cerr << "222222 fpgaRevision2: " << *bar2.fpgaRevision2 << endl;
  cerr << "222222 coreOption: " << *bar2.coreOption << endl;
  cerr << "222222 gblRegControl: " << *bar2.gblRegControl << endl;
  cerr << "222222 masterAControl: " << *bar2.masterAControl << endl;
  cerr << "222222 syncAGen: " << *bar2.syncAGen << endl;
  cerr << "222222 gateAGen: " << *bar2.gateAGen << endl;
  cerr << "222222 masterBControl: " << *bar2.masterBControl << endl;
  cerr << "222222 syncBGen: " << *bar2.syncBGen << endl;
  cerr << "222222 gateBGen: " << *bar2.gateBGen << endl;
  cerr << "222222 syncMask: " << *bar2.syncMask << endl;
  cerr << "222222 sysIntrEnable: " << *bar2.sysIntrEnable << endl;
  cerr << "222222 sysIntrFlag: " << *bar2.sysIntrFlag << endl;
  cerr << "222222 sysIntrStatus: " << *bar2.sysIntrStatus << endl;
  cerr << "222222 appIntrLintEnable[0]: " << *bar2.appIntrLintEnable[0] << endl;
  cerr << "222222 appIntrLintEnable[1]: " << *bar2.appIntrLintEnable[1] << endl;
  cerr << "222222 appIntrLintEnable[2]: " << *bar2.appIntrLintEnable[2] << endl;
  cerr << "222222 appIntrLintEnable[3]: " << *bar2.appIntrLintEnable[3] << endl;
  cerr << "222222 appIntrFlag: " << *bar2.appIntrFlag << endl;
  cerr << "222222 appIntrStatus: " << *bar2.appIntrStatus << endl;
  cerr << "222222 dacSyncBusSel: " << *bar2.dacSyncBusSel << endl;
  cerr << "222222 dacCtrlStat: " << *bar2.dacCtrlStat << endl;
  
  cerr << "222222 dacFifo.FifoCtrl: " << *bar2.dacFifo.FifoCtrl << endl;
  //P7142_LSB_MSB_PAIR_REG_ADDR  dacFifo.FifoTrigLen;           /* ADC & DDC only */
  cerr << "222222 dacFifo.FifoIntrMask: " << *bar2.dacFifo.FifoIntrMask << endl;
  cerr << "222222 dacFifo.FifoAELevel: " << *bar2.dacFifo.FifoAELevel << endl;
  cerr << "222222 dacFifo.FifoAFLevel: " << *bar2.dacFifo.FifoAFLevel << endl;
  cerr << "222222 dacFifo.FifoStatus: " << *bar2.dacFifo.FifoStatus << endl;
  // P7142_LSB_MSB_PAIR_REG_ADDR dacFifo.FifoPostTrigDlyLength; /* ADC & DDC only */
  // P7142_LSB_MSB_PAIR_REG_ADDR dacFifo.FifoPreTrigCountCapt;  /* ADC only */
  cerr << "222222 &dacFifo.FifoDecimationDivide: " << hex << bar2.dacFifo.FifoDecimationDivide << dec << endl;
  
  cerr << "222222 dac5686Read: " << *bar2.dac5686Read << endl;
  cerr << "222222 dac5686Write: " << *bar2.dac5686Write << endl;
  // P7142_DDR_MEM_REG_ADDR    ddrMem;
  // P7142_FIFO_CTRL_REG_ADDR  ddrMemReadFifo;
  // P7142_FIFO_CTRL_REG_ADDR  ddrMemWriteFifo;
  cerr << "222222 adcSyncBusSel: " << *bar2.adcSyncBusSel << endl;
  // P7142_FIFO_CTRL_REG_ADDR  adcFifo[4];
  // P7142_FIFO_CTRL_REG_ADDR  gbLinkInFifo;
  // P7142_FIFO_CTRL_REG_ADDR  gbLinkOutFifo;
  // P7142_FIFO_CTRL_REG_ADDR  testFifo;
#ifdef P7142_DDC_CORE
  cerr << "222222 coreDdcBase: " << *bar2.coreDdcBase << endl;
  cerr << "222222 coreFiltAccess: " << *bar2.coreFiltAccess << endl;
  cerr << "222222 coreDdcFiltBase: " << *bar2.coreDdcFiltBase << endl;
#endif
#ifdef P7142_INTERP_CORE
  cerr << "222222 coreInterpBase: " << *bar2.coreInterpBase << endl;
#endif
  
#endif
