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
#include "FilterSpec.h"
#include <algorithm>
#include <locale>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <logx/Logging.h>
LOGGING("FilterSpec");

///////////////////////////////////////////////////////////////////////////////
FilterSpec::FilterSpec():
  _ok(true),
  _filePath("") {
}

///////////////////////////////////////////////////////////////////////////////
//
// Constructor reads in the file
//
// If the result is a 0-length vector, then ok will be set false

FilterSpec::FilterSpec(std::string filePath):
        _ok(false),
        _filePath(filePath),
        _isSymmetric(false)
{

  // check for zero length path - indicates
  // that the built-in filters are used instead
  
  if (_filePath.length() == 0) {
    DLOG << "No file path specified, empty filter";
    return;
  }

  // set filter name from file name

  size_t lastSlash = _filePath.find_last_of("/");
  if (lastSlash == std::string::npos) {
    _filterName = _filePath;
  } else {
    _filterName = _filePath.substr(lastSlash  + 1);
  }
  DLOG << "File path: " << _filePath;
  DLOG << "Filter name: " << _filterName;
  
  // open file
  
  FILE *in;
  if ((in = fopen(_filePath.c_str(), "r")) == NULL) {
    int errNum = errno;
    ELOG << "Opening file: " << _filePath;
    ELOG << strerror(errNum);
    return;
  }

  // the file contains a series of numbers, nothing else
  // so use fscanf to read it in

  std::vector<int> ivals;
  while (!feof(in)) {
    double val;
    if (fscanf(in, "%lg", &val) == 1) {
      int ival = (int) (val + 0.5);
      ivals.push_back(ival);
    }
  }
  
  // close file
  
  fclose(in);
  
  // get filter size

  int nTotal = ivals.size();
  int nHalf = nTotal / 2;
  int nCoeffs = nHalf;
  if (ivals.size() % 2 == 1) {
    // odd number of vals
    nCoeffs++;
  }

  // check for symmetric filter

  _isSymmetric = true;
  for (int ii = 0; ii < nHalf; ii++) {
    if (ivals[ii] != ivals[nTotal - 1 - ii]) {
      _isSymmetric = false;
      break;
    }
  }

  if (!_isSymmetric) {

    // if not symmetric, use all the values
    
    for (size_t ii = 0; ii < ivals.size(); ii++) {
      push_back(ivals[ii]);
    }

  } else {

    // for symmetric filtes, use half of the values

    for (int ii = 0; ii < nCoeffs; ii++) {
      push_back(ivals[ii]);
    }

  }
  
  // check for good filter

  if (this->size() > 0) {
    _ok = true;
  }

}

///////////////////////////////////////////////////////////////////////////////
FilterSpec::FilterSpec(std::vector<unsigned int> coefficients) {

    for (unsigned int i = 0; i < coefficients.size(); i++)
	    this->push_back(coefficients[i]);
//    dump();  // for diagnostics
	_ok = true;
}
///////////////////////////////////////////////////////////////////////////////
FilterSpec::~FilterSpec() {
}

///////////////////////////////////////////////////////////////////////////////
bool
FilterSpec::ok() {
  return _ok;
}

///////////////////////////////////////////////////////////////////////////////
std::string
FilterSpec::name() {
  return _filterName;
}

///////////////////////////////////////////////////////////////////////////////
std::string
FilterSpec::path() {
  return _filePath;
}

///////////////////////////////////////////////////////////////////////////////
bool
FilterSpec::isSymmetric() {
  return _isSymmetric;
}

///////////////////////////////////////////////////////////////////////////////
std::string 
  FilterSpec::toStr() 
{

  std::ostringstream sstr;
  sstr << "=========== filter details ============" << std::endl;
  sstr << " filePath: " << _filePath << std::endl;
  sstr << " filterName: " << _filterName << std::endl;
  sstr << " isSymmetric: " << (_isSymmetric?"Y":"N") << std::endl;
  for (unsigned int ii = 0; ii < this->size(); ii++) {
    sstr << "   index, val: " 
         << ii << ", " << (*this)[ii] << std::endl;
  }
  return sstr.str();
  
}

///////////////////////////////////////////////////////////////////////////////
void
  FilterSpec::dump(std::ostream &out) 
{
  out << toStr();
}

void
  FilterSpec::dump()
{
  dump(std::cout);
}

