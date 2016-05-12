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
#ifndef FILTERSPECH_
#define FILTERSPECH_

#include <vector>
#include <string>
#include <ostream>

/// Specify a filter simply as a vector of integer coefficients. 
/// The coefficients can be read from a file, using the constructor
/// which specifies a file name, or the user can populate the
/// vector of coefficients using the alternative constructor.

class FilterSpec : public std::vector<unsigned int> {

public:
	/// Constructor
	/// Read the filter specification from 
	/// from a file.
	FilterSpec(std::string filePath ///< The file to read the values from
			);

	/// Initialize the filter with a vector of coefficients.
	/// @param coefficients The filter coefficients.
	FilterSpec(std::vector<unsigned int> coefficients);

	///  Create an empty FilterSpec
	FilterSpec();

	/// Destructor
	virtual ~FilterSpec();

	/// @return True if the filter specification was acceptible,
	/// false otherwise.
	bool ok();

	/// @return The filter file path
	std::string path();

	/// @return The filter name
	std::string name();

	/// @return true if symmetric
	bool isSymmetric();

	/// print the filter characteristics
        void dump(std::ostream &out);
        void dump(); // to stdout

        /// convert to string representation
        std::string toStr();

protected:
	bool _ok; ///< true if the filter was specified correctly
	std::string _filePath; ///< The path of the file specifying the filter
	std::string _filterName; ///< A filter name - normally the file name
        bool _isSymmetric;
};

#endif
