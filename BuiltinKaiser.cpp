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
#include "BuiltinKaiser.h"

// Kaiser filter coefficients for the kaiser filter in the decimate by 8 down converter
unsigned int ddc8_k5[] = {
		12, 13, 13, 11, 6, 262143, 262133, 262120, 262105, 262088,
		262071, 262053, 262038, 262027, 262021, 262022, 262032, 262052, 262083, 262125,
		33, 94, 161, 233, 303, 369, 425, 466, 486, 481,
		447, 382, 282, 150, 262132, 261943, 261735, 261517, 261298, 261092,
		260911, 260770, 260683, 260662, 260721, 260869, 261113, 261457, 261903, 302,
		936, 1649, 2426, 3248, 4094, 4941, 5764, 6540, 7244, 7854,
		8350, 8717, 8942, 9018
		};

// Filter coefficients for the kaiser filter in the decimate by 4 down converter
unsigned int ddc4_k5[] = {
		262131, 262120, 262110, 262104, 262105, 262117, 262143, 36, 82, 129,
		167, 185, 171, 119, 27, 262046, 261903, 261764, 261658, 261612,
		261650, 261786, 262019, 186, 537, 875, 1138, 1264, 1199, 913,
		408, 261863, 261067, 260273, 259613, 259227, 259246, 259771, 260857, 354,
		2483, 4967, 7622, 10231, 12565, 14411, 15594, 16002
		};

// Kaiser filter coefficients for the kaiser filter in the decimate by 10 down converter
unsigned int ddc10_k5[] = {262136, 262135, 262135, 262135, 262136, 262138, 262140, 
        0, 4, 9, 15, 21, 28, 35, 42, 48, 53, 58, 60, 61, 59, 55, 48, 38, 24, 8,
        262134,  262113, 262090, 262066, 262042, 262018, 261996, 261976, 261959, 
        261947, 261941, 261940, 261947, 261962, 261984, 262015, 262054, 262100,
        8, 66, 128, 193, 258, 321, 379, 432, 475, 506, 524, 527, 512, 479, 426,
        355, 264, 156, 31, 262038, 261889, 261735, 261578, 261423, 261277, 261143,
        261028, 260938, 260877, 260851, 260865, 260921, 261025, 261177, 261379,
        261632, 261934, 139, 532, 965, 1432, 1927, 2442, 2970, 3500, 4025, 4536,
        5023, 5476, 5889, 6253, 6560, 6806, 6985, 7094, 7130 
        };

/////////////////////////////////////////////////////////////////////////////
BuiltinKaiser::BuiltinKaiser() {
	init("ddc8_5_0", ddc8_k5, sizeof(ddc8_k5) / sizeof(unsigned int));
	init("ddc4_4_0", ddc4_k5, sizeof(ddc4_k5) / sizeof(unsigned int));
    init("ddc10_5_0", ddc10_k5, sizeof(ddc10_k5) / sizeof(unsigned int));
}

