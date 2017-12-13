/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzQtVTKViewWin32ExportHeader_h
#define goicpzQtVTKViewWin32ExportHeader_h

/**
* \file goicpzQtVTKViewWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(GOICPZ_STATIC)
  #ifdef GOICPZ_QTVTKVIEW_WINDOWS_EXPORT
    #define GOICPZ_QTVTKVIEWWINEXPORT __declspec(dllexport)
  #else
    #define GOICPZ_QTVTKVIEWWINEXPORT __declspec(dllimport)
  #endif
#else
  #define GOICPZ_QTVTKVIEWWINEXPORT
#endif

#endif
