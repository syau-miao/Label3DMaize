#ifndef _BAGON_ASSERT_H_
#define _BAGON_ASSERT_H_

#include "mex.h"

#define assert(x) if (!(x)) { mexErrMsgTxt(#x);}

#define warn(x) {mexWarnMsgTxt((x));}

#endif // _BAGON_ASSERT_H_
