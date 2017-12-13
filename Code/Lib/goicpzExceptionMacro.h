/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzExceptionMacro_h
#define goicpzExceptionMacro_h

#include "goicpzException.h"

#define goicpzExceptionThrow() throw goicpz::Exception(__FILE__,__LINE__)

#endif
