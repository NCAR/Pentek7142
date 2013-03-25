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
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>
#include <logx/Logging.h>
#include <toolsa/pmu.h>

LOGGING("toggleP7142LEDs")

// For configuration management

#include <QtConfig.h>

#include "p7142sd3c.h"
#include "p7142Up.h"

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

int _chans = 2; ///< number of channels
double _waitSecs = 5.0;  ///< Wait between ids (secs)

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
    ("nChans", po::value<int>(&_chans), "No. of channels (default 2)")
    ("waitSecs", po::value<double>(&_waitSecs), 
     "Wait between toggles (secs) (default 5)")
    ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv)
            .options(descripts).positional(pd).run(), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << "Usage: " << argv[0] << 
      " [OPTION]..." << std::endl;
    std::cout << descripts << std::endl;
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
  
  // Find and open the next PTK714X device
  // user will be asked to pick the device num
  
  volatile unsigned int  cntr;
  volatile unsigned int  dataVal;
  unsigned int           imageSize;
  volatile unsigned int  regVal;
  volatile unsigned int  retVal;
  volatile unsigned int  *virtexReg;
  
  /* Initialize the library */
  DWORD dwStatus = PTK714X_LibInit();
  if (PTK714X_STATUS_OK != dwStatus) {
    return (exitHandler (2, NULL, BootProgram, infile));
  }
  
  /* Find and open the 7142 */
  DWORD BAR0Base;
  DWORD BAR2Base;
  DWORD slot = -1; // forces user interaction if more than 1 pentek
  void *hDev = PTK714X_DeviceFindAndOpen(&slot, &BAR0Base, &BAR2Base);
  if (hDev == NULL) {
    return (exitHandler (3, hDev, BootProgram, infile));
  }

  /* Initialize the 7142 register address table */
  P7142_REG_ADDR p7142Regs;  /* 7142 register table */
  P7142InitRegAddr(BAR0Base, BAR2Base, &p7142Regs);

  return (exitHandler (0, hDev));
  
}

