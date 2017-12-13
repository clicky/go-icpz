/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzWin32ExportHeader_h
#define goicpzWin32ExportHeader_h

/**
* \file goicpzWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(GOICPZ_STATIC)
  #ifdef GOICPZ_WINDOWS_EXPORT
    #define GOICPZ_WINEXPORT __declspec(dllexport)
  #else
    #define GOICPZ_WINEXPORT __declspec(dllimport)
  #endif  /* GOICPZ_WINEXPORT */
#else
/* linux/mac needs nothing */
  #define GOICPZ_WINEXPORT
#endif

#endif
