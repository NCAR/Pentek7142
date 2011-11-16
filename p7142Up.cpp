#include "p7142Up.h"
#include "DDCregisters.h"

#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142Up::p7142Up(p7142 * p7142ptr,
        double sampleClockHz, double ncoFreqHz, char cmMode):
        _p7142ptr(*p7142ptr),
        _sampleClockHz(sampleClockHz),
        _ncoFreqHz(ncoFreqHz),
        _cmMode(cmMode),
        _interp(2),
        _mem2depth(0),
        _mutex()
{
    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    if (isSimulating())
        return;

    // Note that most of the DAC related initialization is
    // performed in p71xx().

    // Initialize the DAC configuration registers
    initDAC();

    return;

}

////////////////////////////////////////////////////////////////////////////////////////
p7142Up::~p7142Up() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
}

////////////////////////////////////////////////////////////////////////////////////////
bool p7142Up::initDAC() {

	// Version: set FIR1 to low pass on DAC ChA and ChB, also disable DAC B, if operating at 48 or 125 MHz
    char version =
            1 << 7 |              // DAC A sleep
            0 << 6 |              // DAC B operational
            0 << 5 |              // hplb, DAC B fir1
            0 << 4;               // hpla, DAC A fir1
    setDACreg(DAC5687_VERSION_REG, version);

    // Config 0:
    // Bypass the internal DAC FIFOs since we are using PLL.
    // Set the NCO configuration to high freq,
    // PLL divider = 1,
    // operation mode = X4L.

    char config0;

    switch (int(_sampleClockHz)) {
    case 125000000:
        config0 =
                0 << 6 |               // pll_div
                1 << 5 |               // pll_freq
                0 << 4 |               // pll_kv
                _interp << 2  |        // interpolation mode
                0 << 1 |               // inv_pllock
                1 << 0;                // fifo_bypass
        break;
    case 100000000:
        std::cerr << std::endl;
        std::cerr << "NEED UPCONVERTER SETUP FOR 100 MHz CLOCK!" << std::endl;
        std::cerr << std::endl;
        config0 = _interp << 2;
        break;
    case 48000000:
        config0 =
                1 << 6 |               // pll_div
                0 << 5 |               // pll_freq
                0 << 4 |               // pll_kv
                _interp << 2  |    // interp
                0 << 1 |               // inv_pllock
                1 << 0;                // fifo_bypass
        break;
    default:
        std::cerr << __PRETTY_FUNCTION__ <<
            " has no handling for sample clock @ " << _sampleClockHz <<
            " Hz!" << std::endl;
        exit(1);
    }

    setDACreg(DAC5687_CONFIG0_REG, config0);

    // Config 1: Set input Data two two's complement, non-interleaved
    char config1 = 1 << 4;
    setDACreg(DAC5687_CONFIG1_REG, config1);

    // Config 2: Enable NCO, set cm_mode, enable inv. sync filter
    char config2 = 0x80 | (_cmMode << 1) | 0x1;
    setDACreg(DAC5687_CONFIG2_REG, config2);

    // Config 3: For now just a placeholder
    char config3 = 0x80;
    setDACreg(DAC5687_CONFIG3_REG, config3);

    // Sync Control: Sync NCO, sync coarse mixer, disable FIFO sync
    char sync_cntl = 0x40 | 0x20 | 0x6 << 2;

    setDACreg(DAC5687_SYNC_CNTL_REG, sync_cntl);

    char nco_0;
    char nco_1;
    char nco_2;
    char nco_3;
    ncoConfig(_ncoFreqHz, 4*_sampleClockHz, nco_0, nco_1, nco_2, nco_3);
    std::cout << std::hex <<
            (int)nco_0 << " " <<
            (int)nco_1 << " " <<
            (int)nco_2 << " " <<
            (int)nco_3 << " " <<
            std::dec << std::endl;

    setDACreg(DAC5687_NCO_FREQ_0_REG, nco_0);
    setDACreg(DAC5687_NCO_FREQ_1_REG, nco_1);
    setDACreg(DAC5687_NCO_FREQ_2_REG, nco_2);
    setDACreg(DAC5687_NCO_FREQ_3_REG, nco_3);

    std::cout << "DAC registers after configuration " << std::endl;
    dumpDACregs();

    std::cout << "sample clock:     " << _sampleClockHz << std::endl;
    std::cout << "nco frequency:    " << _ncoFreqHz << std::endl;
    std::cout << "coarse mixer mode:" << (int)_cmMode << std::endl;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p7142Up::isSimulating() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return _p7142ptr.isSimulating();
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::dumpDACregs() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
        std::cout << "No DAC registers: running in simulation mode." << std::endl;
        return;
    }

    for (int i = 0; i < 32; i++) {
        // get value
        char val = getDACreg(i);
        std::cout << "DAC register 0x"  << std::hex  << i << std::dec << ":";
        // print binary
        for (int i = 0; i < 8; i++) {
            char mask = 1 << (7 - i);
            std::cout << " ";
            std::cout << ((val & mask)? "1":"0");
        }
        // print hex
        std::cout << "  " << std::hex << (((int)val) & 0xff) << std::dec << "     ";
        std::cout << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
char
p7142Up::getDACreg(int reg) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return 0;

    char val = (char)P7142Dac5686ReadReg (&_p7142ptr._p7142Regs.BAR2RegAddr, reg);

    return val;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::setDACreg(int reg, char val) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    P7142Dac5686WriteReg (&_p7142ptr._p7142Regs.BAR2RegAddr, reg, val);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::write(int32_t* data, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // memory depth in 4 byte words
    _mem2depth = n;

    P7142_SET_DDR_MEM_DEPTH(
    		_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemBankDepth[2].Lsb,
    		_mem2depth);

    // Transfer data to memory 2
    int writeStatus = _p7142ptr.ddrMemWrite(
    		&_p7142ptr._p7142Regs,
    		P7142_DDR_MEM_BANK2,
            0,
            n*4,
            data,
            _p7142ptr._deviceHandle);

    if (writeStatus) {
    	std::cerr << "DMA write to memory bank 2 failed, with status code: " << writeStatus << std::endl;
    }

}


////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::startDAC() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // Set DDR memory for output to DAC. This will clear and then set
    // bits D06 and D10 in the DDR memory control.
    // Resetting D06 has the side effect of zeroing the memory counter.
    P7142_SET_DDR_MEM_MODE(
    		_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl,
     		P7142_DDR_MEM_BANK_2_DAC_OUTPUT_MODE);

     // Enable the DAC memory FIFO. This sets the FIFO enable bit D0
     P7142_SET_FIFO_CTRL_FIFO_ENABLE(
    		_p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
    		P7142_FIFO_ENABLE);

     // Place the DAC memory FIFO into reset by setting bit D1.
     P7142_SET_FIFO_CTRL_RESET(
    		 _p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
    		 P7142_FIFO_RESET_HOLD);

     // Place the DAC memory FIFO in run mode by clearing bit D1. The FIFO
     // will be clocked when the tx_gate allows the memory counter
     // to run.
     P7142_SET_FIFO_CTRL_RESET(
    		 _p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
    		 P7142_FIFO_RESET_RELEASE);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::stopDAC() {
  boost::recursive_mutex::scoped_lock guard(_mutex);

  if (isSimulating())
      return;

  // Turn off data routing from mem2
  P7142_SET_DDR_MEM_MODE(
  		_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl,
  		P7142_DDR_MEM_DISABLE_MODE);

  // disable the NCO in order to stop the DAC
  char config2 = (_cmMode << 1) | 0x1;
  setDACreg(DAC5687_CONFIG2_REG, config2);

}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::ncoConfig(double fNCO, double fDAC, char& nco_freq_0, char& nco_freq_1, char& nco_freq_2, char& nco_freq_3) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;
    
    double fNCO_CLK;

    switch (_interp) {
    default:
    case 0: // X2  mode
    case 2: // X4L mode
    case 3: // X8  mode
        fNCO_CLK = fDAC / 2;;
        break;
    case 1: // X4 mode
        fNCO_CLK = fDAC;
        break;
    }

    long long freq;

    if ((fNCO/fNCO_CLK) < 0.5)
        freq = (long long)((fNCO/fNCO_CLK)*(0x100000000ll));
    else
        /// @todo the following produces a 33 bit number! There is something
        /// wrong with the formula in the DAC datasheet.
        freq = (long long)(((fNCO/fNCO_CLK)+1)*(0x100000000ll));

    nco_freq_0 = (freq >>  0) & 0xff;
    nco_freq_1 = (freq >>  8) & 0xff;
    nco_freq_2 = (freq >> 16) & 0xff;
    nco_freq_3 = (freq >> 24) & 0xff;

    // std::cout << "freq is " << std::hex << freq << std::dec << std::endl;

}

////////////////////////////////////////////////////////////////////////////////////////
double
p7142Up::sampleClockHz() {
	return _sampleClockHz;
}

