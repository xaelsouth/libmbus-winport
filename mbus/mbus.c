//------------------------------------------------------------------------------
// Copyright (C) 2010, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#include "mbus-protocol.h"
#ifndef _WIN32
#include "../config.h"
#endif

//
//
//
ADDAPI int ADDCALL mbus_init() {return 0;}

///
/// Return current version of the library
///
ADDAPI const char* ADDCALL
mbus_get_current_version() {return PACKAGE_VERSION;}
