// pch.h : primary include file for standard system includes

#pragma once

#include <ctime>
#include <cstring>
#include <sstream>
#include <string>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include <boost/geometry.hpp>

namespace spatial {
    using namespace std;

    using namespace boost::geometry;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef TO_DEGREES 
#define TO_DEGREES 57.295779513082321
#endif

#ifndef TO_RADIANS
#define TO_RADIANS 0.0174532925199432958
#endif

#ifdef __GNUC__
#define NO_RETURN __attribute__((__noreturn__))
#else
#define NO_RETURN
#endif

#ifndef uassert
#define uassert(msgid, msg, expr) (void)( (bool)(!!(expr)) || (NORETURN, 0) )
#endif

#undef NORETURN
}
