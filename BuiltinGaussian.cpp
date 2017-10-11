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
#include "BuiltinGaussian.h"
#include <stdint.h>

// Gaussian filter coefficients for the decimate by 8 down converter
uint32_t ddc8_0_2[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 3, 43, 395, 2316,
		8730, 21144, 32904 };

uint32_t ddc8_0_3[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 3, 19, 92, 365, 1186, 3168, 6955,
		12543, 18584, 22620 };

uint32_t ddc8_0_5[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 3, 12, 39, 118, 320, 776, 1682, 3266, 5676, 8833,
		12308, 15354, 17149 };
uint32_t ddc8_0_6[] = { 0, 0, 0, 0, 0, 0, 0, 1, 3,
8, 22, 55, 129, 281, 571, 1079, 1901, 3120, 4770, 6795, 9018,
11150, 12845, 13787 };

uint32_t ddc8_0_7[] = { 0, 0, 0, 0, 1, 3, 6, 14, 32,
		66, 132, 249, 450, 772, 1262, 1964, 2910, 4105, 5513, 7048, 8579,
		9941, 10968, 11521 };

uint32_t ddc8_0_8[] = { 0, 1, 2, 5, 10, 21, 39, 73, 130,
		223, 370, 592, 913, 1358, 1948, 2696, 3599, 4634, 5755, 6894, 7965,
		8876, 9540, 9891 };

uint32_t ddc8_1_0[] = { 4, 8, 15, 26, 45, 77, 126, 202, 314,
		476, 700, 1003, 1398, 1894, 2498, 3203, 3996, 4849, 5723, 6572, 7340,
		7975, 8428, 8664 };

// Gaussian filter coefficients for the decimate by 4 down converter
uint32_t ddc4_0_500[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 2, 11, 59, 264,
		952, 2773, 6524, 12394, 19011, 23545 };

uint32_t ddc4_0_667[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 2, 6, 24, 80, 236, 617, 1431,
		2943, 5368, 8682, 12452, 15835, 17858 };

uint32_t ddc4_1_000[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 3, 8,
		20, 44, 93, 185, 352, 634, 1083, 1752, 2687, 3907,
		5385, 7036, 8714, 10231, 11385, 12011 };

uint32_t ddc4_1_333[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 2, 4, 9, 16, 30, 53, 90, 151,
		244, 383, 584, 864, 1240, 1727, 2333, 3058, 3891, 4803,
		5754, 6688, 7544, 8257, 8769, 9037 };

uint32_t ddc4_2_667[] = {
		40, 52, 67, 86, 109, 138, 172, 214, 265, 324,
		394, 476, 570, 677, 799, 936, 1088, 1255, 1436, 1632,
		1841, 2061, 2289, 2524, 2763, 3001, 3235, 3462, 3676, 3875,
		4054, 4209, 4337, 4436, 4504, 4538 };
        
// Gaussian filter coefficients for the decimate by 4 down converter for use when pulse coding is active.
uint32_t ddc4_0_500_pulsecode[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 2, 19, 116,
		543, 1964, 5492, 11873, 19852, 25670 };

uint32_t ddc4_0_667_pulsecode[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 7, 29, 108, 342, 940,
		2236, 4604, 8204, 12654, 16891, 19516 };

uint32_t ddc4_1_000_pulsecode[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 2,
		6, 15, 38, 88, 189, 384, 730, 1301, 2175, 3411,
		5015, 6915, 8942, 10843, 12330, 13148 };

uint32_t ddc4_1_333_pulsecode[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 2, 5, 10, 20, 39, 72,
		129, 222, 369, 590, 910, 1355, 1945, 2693, 3596, 4632,
		5754, 6895, 7968, 8881, 9547, 9898 };

uint32_t ddc4_2_667_pulsecode[] = {
		17, 23, 31, 42, 56, 74, 98, 127, 163, 208,
		264, 331, 411, 505, 616, 745, 893, 1060, 1247, 1454,
		1680, 1924, 2183, 2455, 2736, 3022, 3308, 3588, 3857, 4109,
		4338, 4538, 4705, 4834, 4922, 4967 };

// DDC10 original filter for 500 ns transmit pulse
uint32_t ddc10_0_500[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 10, 70, 385, 1584, 4914, 11487, 20233, 26853
    };

// DDC10 flat passband filter for 500 ns transmit pulse
uint32_t ddc10_0_500_flat[] = {
        262125, 262080, 262044, 262042, 262095, 61, 198, 301, 299,
        139, 261975, 261610, 261349, 261374, 261793, 418, 1301, 1907,
        1828, 827, 261161, 259077, 257593, 257677, 260039, 2667, 9240,
        16338, 22348, 25793
    };

// DDC10 filter for 1000 ns transmit pulse
uint32_t ddc10_1_000[] = {
        10, 36, 71, 113, 156, 195, 218, 213, 168, 70, 262059, 261844,
        261583, 261296, 261022, 260808, 260713, 260797, 261120, 261726,
        497, 1719, 3214, 4914, 6721, 8515, 10164, 11539, 12526, 13042
    };

/////////////////////////////////////////////////////////////////////////////
BuiltinGaussian::BuiltinGaussian() {
	init("ddc8_0_2", ddc8_0_2, sizeof(ddc8_0_2) / sizeof(uint32_t));
	init("ddc8_0_3", ddc8_0_3, sizeof(ddc8_0_3) / sizeof(uint32_t));
	init("ddc8_0_5", ddc8_0_5, sizeof(ddc8_0_5) / sizeof(uint32_t));
	init("ddc8_0_6", ddc8_0_6, sizeof(ddc8_0_6) / sizeof(uint32_t));
	init("ddc8_0_7", ddc8_0_7, sizeof(ddc8_0_7) / sizeof(uint32_t));
	init("ddc8_0_8", ddc8_0_8, sizeof(ddc8_0_8) / sizeof(uint32_t));
	init("ddc8_1_0", ddc8_1_0, sizeof(ddc8_1_0) / sizeof(uint32_t));

	init("ddc4_0_500", ddc4_0_500, sizeof(ddc4_0_500) / sizeof(uint32_t));
	init("ddc4_0_667", ddc4_0_667, sizeof(ddc4_0_667) / sizeof(uint32_t));
	init("ddc4_1_000", ddc4_1_000, sizeof(ddc4_1_000) / sizeof(uint32_t));
	init("ddc4_1_333", ddc4_1_333, sizeof(ddc4_1_333) / sizeof(uint32_t));
	init("ddc4_2_667", ddc4_2_667, sizeof(ddc4_2_667) / sizeof(uint32_t));
    
	init("ddc4_0_500_pulsecode", ddc4_0_500_pulsecode, sizeof(ddc4_0_500_pulsecode) / sizeof(uint32_t));
	init("ddc4_0_667_pulsecode", ddc4_0_667_pulsecode, sizeof(ddc4_0_667_pulsecode) / sizeof(uint32_t));
	init("ddc4_1_000_pulsecode", ddc4_1_000_pulsecode, sizeof(ddc4_1_000_pulsecode) / sizeof(uint32_t));
	init("ddc4_1_333_pulsecode", ddc4_1_333_pulsecode, sizeof(ddc4_1_333_pulsecode) / sizeof(uint32_t));
	init("ddc4_2_667_pulsecode", ddc4_2_667_pulsecode, sizeof(ddc4_2_667_pulsecode) / sizeof(uint32_t));

    init("ddc10_0_500", ddc10_0_500, sizeof(ddc10_0_500) / sizeof(uint32_t));
    init("ddc10_0_500_flat", ddc10_0_500_flat, sizeof(ddc10_0_500_flat) / sizeof(uint32_t));
    init("ddc10_1_000", ddc10_1_000, sizeof(ddc10_1_000) / sizeof(uint32_t));
}

