/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzQtVTKControllerWin32ExportHeader_h
#define goicpzQtVTKControllerWin32ExportHeader_h

/**
* \file goicpzQtVTKControllerWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(GOICPZ_STATIC)
  #ifdef GOICPZ_QTVTKCONTROLLER_WINDOWS_EXPORT
    #define GOICPZ_QTVTKCONTROLLERWINEXPORT __declspec(dllexport)
  #else
    #define GOICPZ_QTVTKCONTROLLERWINEXPORT __declspec(dllimport)
  #endif
#else
  #define GOICPZ_QTVTKCONTROLLERWINEXPORT
#endif

#endif
