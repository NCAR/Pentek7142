// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=* 
// ** Copyright UCAR (c) 1990 - 2016                                         
// ** University Corporation for Atmospheric Research (UCAR)                 
// ** National Center for Atmospheric Research (NCAR)                        
// ** Boulder, Colorado, USA                                                 
// ** BSD licence applies - redistribution and use in source and binary      
// ** forms, with or without modification, are permitted provided that       
// ** the following conditions are met:                                      
// ** 1) If the software is modified to produce derivative works,            
// ** such modified software should be clearly marked, so as not             
// ** to confuse it with the version available from UCAR.                    
// ** 2) Redistributions of source code must retain the above copyright      
// ** notice, this list of conditions and the following disclaimer.          
// ** 3) Redistributions in binary form must reproduce the above copyright   
// ** notice, this list of conditions and the following disclaimer in the    
// ** documentation and/or other materials provided with the distribution.   
// ** 4) Neither the name of UCAR nor the names of its contributors,         
// ** if any, may be used to endorse or promote products derived from        
// ** this software without specific prior written permission.               
// ** DISCLAIMER: THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS  
// ** OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED      
// ** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.    
// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=* 
 /**********************************************************************************************

Desc: Pentek 7142 HCR DDC Registers

**********************************************************************************************/

/// ADC FIFO Control Registers
#define ADC_FIFO_CTRL_1 0x8C08  ///< ADC #1
#define ADC_FIFO_CTRL_2 0x8C40  ///< ADC #2
#define ADC_FIFO_CTRL_3 0x8C78  ///< ADC #3
#define ADC_FIFO_CTRL_4 0x8CB0  ///< ADC #4

/// The DAC FIFO Control Register
#define DAC_FIFO_CONTROL 0x8410

/// The DDR Memory Control Register
#define DDR_MEM_CONTROL 0x8800

/// Pentek DCM control register
#define DCM_CONTROL    0x8020   ///< Pentek Gateflow defined register, used to reset the digital clock managers.

/// GateFlow option 223 ttl_in register
#define TTL_IN          0x8200  ///< Partial implementation of the Pentek Gateflow Option 223 allows us to perform TTL input via this register.

/// GateFlow option 223 ttl_out1 register, 16 bits
#define TTL_OUT1        0x8208  ///< Partial implementation of the Pentek Gateflow Option 223 allows us to perform TTL output via this register.

// FRAP Transceiver control register
#define TRANS_CNTRL     0x8E78  ///< Transceiver firmware control register

// MultiTimer registers
#define MT_ADDR         0x8E80  ///< Multi-timer address and control
#define MT_DATA         0x8E88  ///< Multi-timer data
#define MT_WR           0x8E90  ///< Multi-timer write strobes

// Kaiser filter registers
#define KAISER_ADDR     0x8E98  ///< Register 467 Kaiser filter coefficient address and control
#define KAISER_DATA_LSW 0x8EA0  ///< Register 468 Kaiser filter coefficient data (LSW)
#define KAISER_DATA_MSW 0x8EA8  ///< Register 469 Kaiser filter coefficient data (MSW)
#define KAISER_WR       0x8EB0  ///< Register 470 Kaiser filter write
#define KAISER_READ_LSW 0x8EB8  ///< Register 471 Kaiser filter coefficient read (LSW)
#define KAISER_READ_MSW 0x8EC0  ///< Register 472 Kaiser filter coefficient read (MSW)

// Gaussian filter registers
#define GAUSSIAN_ADDR     0x8EC8  ///< Register 473 Gaussian filter coefficient address and control
#define GAUSSIAN_DATA_LSW 0x8ED0  ///< Register 474 Gaussian filter coefficient data (LSW)
#define GAUSSIAN_DATA_MSW 0x8ED8  ///< Register 475 Gaussian filter coefficient data (MSW)
#define GAUSSIAN_WR       0x8EE0  ///< Register 476 Gaussian filter write
#define GAUSSIAN_READ_LSW 0x8EE8  ///< Register 477 Gaussian filter coefficient read (LSW)
#define GAUSSIAN_READ_MSW 0x8EF0  ///< Register 478 Gaussian filter coefficient read (MSW)

// Radar gates register
#define RADAR_GATES       0x8EF8 ///< Register 479 The number of radar gates

// Number of coherent integrations register
#define CI_NSUM           0x8F00 ///< Register 480 The number of values for one coherent integrator sum

// FPGA source code revision and DDC type register
#define FPGA_REPO_REV     0x8F08 ///< Register 481 The fpga source code repository revision number

//Clock Decimation Registers
//  #define DEC_REG     0xA20
//  #define DEC_RST_REG 0xA24
//  #define RST_ACT     0x1
//  #define RST_CLR     0x0

//Gate Splitter Register
//  #define DPRT_REG 0xA50
//Values
//  #define DPRT_ON  0x1
//  #define DPRT_OFF 0x0

// Transceiver control bits
#define TRANS_FREE_RUN     0x1  ///< Bit mask to access the free-run enable bit in the transceiver control register

// DDC Filter control bits
#define DDC_START          0x0  ///< Allow the filter to run. Set in the filter ADDR register.
#define DDC_STOP     (0x01<<12) ///< Force the filter to stop. Set in the filter ADDR register.

//MultTimer Sub Registers
#define WRITE_ON  0x3
#define WRITE_OFF 0x0

#define CONTROL_REG 0x8
#define DELAY_REG   0x9
#define WIDTH_REG   0xA
#define PERIOD_REG  0xB
#define PRT_REG     0xC

#define W_Ch0 (0x01<<4)
#define W_Ch1 (0x02<<4)
#define W_Ch2 (0x04<<4)
#define W_Ch3 (0x08<<4)
#define K_Ch0 (0x10<<4)
#define K_Ch1 (0x20<<4)
#define K_Ch2 (0x40<<4)
#define K_Ch3 (0x80<<4)

#define GLOBAL_EN  (0x1<<12)
#define GPS_EN	  (0x1<<13)
#define ADDR_TRIG (0x1<<15)

#define TIMER_ON  (0x1<<0)
#define TIMER_OFF (0x0<<0)
#define TIMER_NEG (0x1<<1)
#define TIMER_POS (0x0<<1)
#define CLK_DIV1  (0x0<<2)
#define CLK_DIV2  (0x1<<2)
#define CLK_DIV4  (0x2<<2)
#define CLK_DIV8  (0x3<<2)

#define PRT1 (0x0<<0)
#define PRT2 (0x0<<4)
#define PRT3 (0x0<<8)
#define PRT4 (0x0<<12)

// Timer DCM Lock Status
//  #define FILTER_DCM_UNLOCKED (0x1<<1)
//  #define FILTER_DCM_RST (0x1<<1)

//-------------------------------
//Added by Tom 1-3-08 for timing error detection
//Bits 23-20 of status register correspond to sync error for channel A-D
//#define A_SYNC_ERROR (0x1<<23)
//#define B_SYNC_ERROR (0x1<<22)
//#define C_SYNC_ERROR (0x1<<21)
//b#define D_SYNC_ERROR (0x1<<20)

