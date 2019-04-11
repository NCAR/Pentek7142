/* -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; -*-
 * sd3c_info_7142.cpp
 *
 * Print information about the SD3C bistream (if any) installed on a Pentek
 * 7142 card.
 *
 *  Created on: Aug 4, 2017
 *      Author: Chris Burghart <burghart@ucar.edu>
 */
// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
// ** Copyright UCAR (c) 2017-2017
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
#include <string>
#include <logx/Logging.h>
#include <p7142sd3c.h>

LOGGING("sd3c_info_7142")

using namespace Pentek;

int
main(int argc, char** argv)
{
    // Let logx get and strip out its arguments
    logx::ParseLogArgs(argc, argv);

    p7142sd3c::DDCDECIMATETYPE ddctype = Pentek::p7142sd3c::DDCUNDEFINED;

    std::string ddcname;
    if (argc > 1)
	ddcname = argv[1];

    p7142sd3c::DDCDECIMATETYPE knowntypes[] = {
	p7142sd3c::DDCUNDEFINED, p7142sd3c::DDC10DECIMATE, 
        p7142sd3c::DDC8DECIMATE, p7142sd3c::DDC6DECIMATE, 
        p7142sd3c::DDC4DECIMATE, p7142sd3c::BURST
    };

    for (int i=0; i < sizeof(knowntypes)/sizeof(knowntypes[0]); ++i)
    {
        if (p7142sd3c::ddcTypeName(knowntypes[i]) == ddcname)
        {
            ddctype = knowntypes[i];
            break;
        }
    }

    // Instantiate a p7142sd3c object using the first Pentek 7142 on the system.
    // The constructor will log the SD3C DDC type and revision number.
    Pentek::p7142sd3c dev(false, 0.0, 0.0, 0.001, 0.0, false, 100, 1, false,
			  ddctype, false, 0.0, true, false, 0, 0);
}

